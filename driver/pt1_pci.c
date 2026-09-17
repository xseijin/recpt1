/* -*- tab-width: 4; indent-tabs-mode: t -*- */
/* pt1-pci.c: A PT1 on PCI bus driver for Linux. */
/*
 * FIX: 以前はここで #define DRV_NAME "pt1-pci" していたが、直後の
 * #include "version.h" が DRV_NAME を "pt1_drv" に即座に再定義しており、
 * この行と version.h の間で DRV_NAME は一度も参照されていなかった。
 * つまり完全に無意味な定義で、毎回ビルド時に
 * "DRV_NAME redefined" 警告を出すだけの死んだコードだった。削除する。
 */
#include "version.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/uaccess.h>
#include <linux/compiler.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>

#include <linux/ioctl.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>

#include        "pt1_com.h"
#include        "pt1_pci.h"
#include        "pt1_tuner.h"
#include        "pt1_i2c.h"
#include        "pt1_tuner_data.h"
#include        "pt1_ioctl.h"

/* These identify the driver base version and may not be removed. */
static char version[] =
DRV_NAME ".c: " DRV_VERSION " " DRV_RELDATE " \n";

MODULE_AUTHOR("Tomoaki Ishikawa tomy@users.sourceforge.jp and Yoshiki Yazawa yaz@honeyplanet.jp");
#define DRIVER_DESC             "PCI earthsoft PT1/2 driver"
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static int debug = 7;                   /* 1 normal messages, 0 quiet .. 7 verbose. */
static int lnb = 0;                     /* LNB OFF:0 +11V:1 +15V:2 */
static int dma_cpu = -1;                /* DMA/watchdog thread を固定する CPU 番号。-1=無効
                                         * N100 等 録画専用機では isolcpus と合わせて設定すると効果的。
                                         * 例: modprobe pt1_drv dma_cpu=2 */
static bool dma_rt = false;             /* DMA thread を SCHED_FIFO(low) に昇格するか。
                                         * デフォルト OFF: N100 の 4E-core 環境では RT thread が
                                         * NVMe flush / softirq を詰まらせ逆効果になる場合がある。
                                         * CPU pinning(dma_cpu) だけで十分なことが多い。
                                         * 例: modprobe pt1_drv dma_cpu=2 dma_rt=1 */
/* ring_count: 実行時のスロット数。power of 2 必須。
 * 低メモリ環境や ZFS/Docker と共存する場合は 65536 に下げると ~12MB/ch に削減できる。
 * 例: modprobe pt1_drv ring_count=65536
 * TS_RING_COUNT は後方の #define で定義するため、ここでは暫定値 131072 を直接指定し
 * pt1_pci_init() で TS_RING_COUNT を使って再設定する。*/
static unsigned int ring_count = 131072;
static unsigned int ring_mask  = 131071;        /* ring_count - 1, pt1_pci_init() で更新 */

module_param(debug, int, 0);
module_param(lnb, int, 0);
module_param(dma_cpu, int, 0444);
module_param(dma_rt, bool, 0644);
module_param(ring_count, uint, 0444);
MODULE_PARM_DESC(debug, "debug level (1-2)");
MODULE_PARM_DESC(lnb, "LNB level (0:OFF 1:+11V 2:+15V)");
MODULE_PARM_DESC(dma_cpu, "CPU number to pin DMA/watchdog threads (-1=no pinning, default)");
MODULE_PARM_DESC(dma_rt, "Elevate DMA thread to SCHED_FIFO(low) (default=0/OFF, use with caution on N100)");
MODULE_PARM_DESC(ring_count, "Ring buffer slots per channel (default=131072=~24MB/ch, must be power of 2)");
/* NOTE: 元コードは MODULE_PARM_DESC(debug, ...) が2回あったため lnb 側を修正 */

#define VENDOR_EARTHSOFT 0x10ee
#define PCI_PT1_ID 0x211a
#define PCI_PT2_ID 0x222a

static struct pci_device_id pt1_pci_tbl[] = {
        { VENDOR_EARTHSOFT, PCI_PT1_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
        { VENDOR_EARTHSOFT, PCI_PT2_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
        { 0, }
};
MODULE_DEVICE_TABLE(pci, pt1_pci_tbl);
#define         DEV_NAME        "pt1video"

#define         PACKET_SIZE                     188             // 1パケット長
#define         MAX_READ_BLOCK  4                       // 1度に読み出す最大DMAバッファ数
#define         MAX_PCI_DEVICE          128             // 最大64枚
#define         DMA_SIZE        4096                    // DMAバッファサイズ
#define         DMA_RING_SIZE   128                     // number of DMA RINGS
#define         DMA_RING_MAX    511                     // number of DMA entries in a RING
#define         CHANNEL_DMA_SIZE        (2*1024*1024)   // 地デジ用(16Mbps)
#define         BS_CHANNEL_DMA_SIZE     (4*1024*1024)   // BS用(32Mbps)
#define         READ_SIZE       (16*DMA_SIZE)
/*
 * WAKEUP_WATERMARK: ring にこのスロット数溜まってから consumer を起こす。
 * 256 packets = 256 * 188 = ~48KB。wakeup storm を防ぎ、
 * recisdb の bulk read と相性が良い。
 */
#define         WAKEUP_WATERMARK        256

/*
 * DMA_BUSY: ステータスレジスタ(オフセット 0x00)のbit6。
 * DMA転送中を示す。pt1_pci.h の FIFO_GO_ADDR(0x00) と同じレジスタ。
 * pt1_pci.h には定義がないためここで補完する。
 */
#ifndef DMA_BUSY
#define DMA_BUSY        (1 << 6)
#endif
/* DMA_ADDR (0x14) は pt1_pci.h で定義済み。ここでは定義しない。 */

/* デフォルト値: ring_count の初期値と一致させる */
#define TS_RING_COUNT   131072          /* slots (131072 * 188 = ~24MB/ch, 8ch合計~188MB)
                                         * 8ch同時録画時に約1秒分のstallを吸収できる。
                                         * must be power of 2 */
#define TS_RING_MASK    (TS_RING_COUNT - 1)

/*
 * Circular ring buffer per channel.
 * head: written by DMA kthread (producer)
 * tail: written by pt1_read() (consumer)
 * head/tail are accessed with smp_store_release / smp_load_acquire so that
 * no additional spinlock is needed for the fast path.
 */
struct ts_ring {
        u8      *buffer;                /* vzalloc'd, ring_count * PACKET_SIZE bytes */
        u32     head ____cacheline_aligned;
        u32     tail ____cacheline_aligned;
        /*
         * overflow_lock: tail と別 cacheline に置く。
         * overflow は rare だが、tail は consumer が常時更新するため
         * 同 cacheline にいると false sharing で不要な contention が生じる。
         */
        spinlock_t overflow_lock ____cacheline_aligned;
};

typedef struct  _DMA_CONTROL{
        dma_addr_t      ring_dma[DMA_RING_MAX] ;        // DMA情報
        __u32           *data[DMA_RING_MAX];
}DMA_CONTROL;

typedef struct  _PT1_CHANNEL    PT1_CHANNEL;

typedef struct  _pt1_device{
        unsigned long   mmio_start ;
        __u32                   mmio_len ;
        void __iomem            *regs;
        struct mutex            lock ;
        dma_addr_t              ring_dma[DMA_RING_SIZE] ;       // DMA情報
        void                    *dmaptr[DMA_RING_SIZE] ;
        struct  task_struct     *kthread;
        dev_t                   dev ;
        int                     card_number;
        __u32                   base_minor ;
        struct  cdev    cdev[MAX_CHANNEL];
        wait_queue_head_t       dma_wait_q ;// for poll on reading
        DMA_CONTROL             *dmactl[DMA_RING_SIZE];
        PT1_CHANNEL             *channel[MAX_CHANNEL];
        int                     cardtype;
        unsigned long last_reset;
        /* DMA watchdog */
        atomic64_t              dma_counter ____cacheline_aligned;
                                                /* DMA thread が inc、watchdog が read: 別 cacheline で bouncing 抑制 */
        unsigned long           last_dma_jiffies;
        struct task_struct      *watchdog_task;
        /*
         * dma_ready: DMA が初期化済みで稼働中であることを示すフラグ。
         * suspend 中は false にして watchdog が誤 recover しないようにする。
         * WRITE_ONCE/READ_ONCE でアクセスする。
         */
        bool                    dma_ready;
        /* stats: debugfs / tracepoint 用 */
        atomic64_t              recover_count;  /* pt1_dma_recover 呼び出し回数 */
        atomic64_t              sync_loss;      /* TS sync 喪失回数 */
        /* DMA recover reason 分類 */
        atomic64_t              recover_stall;          /* watchdog stall 検出 */
        atomic64_t              recover_sync_loss;      /* sync loss 起因 */
        atomic64_t              recover_descriptor;     /* descriptor 破損 */
        atomic64_t              recover_timeout;        /* DMA idle timeout */
        /*
         * recovering: pt1_dma_recover の single-flight guard。
         * watchdog と DMA thread が同時に recover に入ると reset_dma が
         * 二重実行されハードウェアが不定状態になる。
         * atomic_xchg で先着1つだけ recover を実行する。
         */
        atomic_t                recovering;
        /*
         * recover backoff / permanent fault:
         * recover 連続失敗時に指数 backoff を適用し、
         * 10回失敗で device_dead へ遷移して無限 recover loop を防ぐ。
         */
        int                     recover_fail_streak;    /* 連続失敗カウント */
        unsigned long           recover_backoff_until;  /* backoff 期限 jiffies */
        bool                    device_dead;            /* true: 永続的障害、recover 停止 */
        /* debugfs */
        struct dentry           *debugfs_dir;
} PT1_DEVICE;

typedef struct  _MICRO_PACKET{
        char    data[3];
        char    head ;
}MICRO_PACKET;

struct  _PT1_CHANNEL{
        __u32                   valid ;                 // 使用中フラグ
        __u32                   address ;               // I2Cアドレス
        __u32                   channel ;               // チャネル番号
        int                     type ;                  // チャネルタイプ
        __u32                   packet_size ;   // パケットサイズ
        atomic_t                drop ;                  // パケットドロップ数（atomic: producer/watchdog/read で競合）
        struct mutex            lock ;                  // CH別mutex_lock用
        __u32                   size ;                  // DMAされたサイズ
        __u32                   maxsize ;               // DMA用バッファサイズ
        __u32                   bufsize ;               // チャネルに割り振られたサイズ
        atomic_t                overflow ;              // オーバーフローエラー発生（atomic: DMA thread/release で競合）
        __u32                   counetererr ;   // 転送カウンタ１エラー
        __u32                   transerr ;              // 転送エラー
        __u32                   minor ;                 // マイナー番号
        __u8                    *buf;                   // CH別受信メモリ (legacy; ring.buffer を参照)
        __u32                   pointer;
        struct ts_ring          ring;                   // circular ring buffer
        __u8                    req_dma ;               // 溢れたチャネル
        __u8                    packet_buf[PACKET_SIZE + 4] ;   // 元版と同じ +4 の余裕が必要
        /*
         * TS sync hunting (Option B):
         * packet_buf(1パケット分)だけでは「188バイト間隔で3連続0x47」の
         * 検証に必要な377バイトを確保できないため、sync loss時だけ
         * 複数パケット分のスクラッチバッファに切り替えて本来の
         * 周期チェックを行う。hunt_buf が満杯になるたびに検証する。
         */
        bool                    sync_hunting ;          // hunting中フラグ
        __u8                    hunt_buf[3 * PACKET_SIZE] ;    // hunting用スクラッチ(3パケット分, 564byte)
        __u32                   hunt_size ;             // hunt_buf の現在の格納バイト数
        PT1_DEVICE              *ptr ;                  // カード別情報
        wait_queue_head_t       wait_q ;        // for poll on reading
        /*
         * ch_recovering: DMA recover 中であることを consumer に通知するフラグ。
         * recover 開始時に true、完了時に false。
         * read() は recover 中に -EAGAIN を返して古い head/tail を読まないようにする。
         * smp_store_release / smp_load_acquire でアクセスする。
         */
        bool                    ch_recovering;
        /* fill level stats (debugfs export 用) */
        u32                     max_fill;               /* ring 最大使用スロット数 */
        atomic_t                overflow_ring;          /* ring full による drop 回数 */
        atomic_t                underrun_count;         /* read() 時に ring 空だった回数 */
        /*
         * ring occupancy histogram: DMA write 時に fill level を 4 段階に記録。
         * [0]: 0〜25%  [1]: 25〜50%  [2]: 50〜75%  [3]: 75〜100%
         * userspace が追いついているかを定量的に確認できる。
         */
        atomic64_t              fill_hist[4];
        /* wakeup rate: wake_up_interruptible_poll を呼んだ回数 */
        atomic64_t              wakeup_count;
        /* wakeup rate 計算用: 最後にリセットした jiffies と count スナップショット */
        unsigned long           wakeup_rate_jiffies;
        u64                     wakeup_rate_snap;
        /* starvation 防止: 最後に wake した jiffies（10Hz fallback 用） */
        unsigned long           last_wake_jiffies;
};

// I2Cアドレス(video0, 1 = ISDB-S) (video2, 3 = ISDB-T)
int             i2c_address[MAX_CHANNEL] = {T0_ISDB_S, T1_ISDB_S, T0_ISDB_T, T1_ISDB_T};
int             real_channel[MAX_CHANNEL] = {0, 2, 1, 3};
int             channeltype[MAX_CHANNEL] = {CHANNEL_TYPE_ISDB_S, CHANNEL_TYPE_ISDB_S,
                                                                        CHANNEL_TYPE_ISDB_T, CHANNEL_TYPE_ISDB_T};

static  PT1_DEVICE      *device[MAX_PCI_DEVICE];
static struct class     *pt1video_class;

#define         PT1MAJOR        251
#define         DRIVERNAME      "pt1video"

/* forward declaration: reset_dma は pt1_dma_recover から呼ばれる */
static void reset_dma(PT1_DEVICE *dev_conf);

/* DMA recover の起因分類 */
enum pt1_recover_reason {
        PT1_RECOVER_STALL       = 0,    /* watchdog: DMA counter 停止 */
        PT1_RECOVER_SYNC_LOSS,          /* TS sync byte 連続エラー */
        PT1_RECOVER_DESCRIPTOR,         /* descriptor NULL / 不正 */
        PT1_RECOVER_TIMEOUT,            /* DMA idle wait timeout */
};

/*
 * pt1_dma_stop - DMAエンジンを停止させる。
 * writelの後 readl でポスト処理し、mb()でメモリバリアを保証する。
 */
static void pt1_dma_stop(PT1_DEVICE *dev)
{
        writel(0x08080000, dev->regs);
        readl(dev->regs);               /* post write flush */
        mb();
}

/*
 * pt1_wait_dma_idle - DMAがアイドルになるまで最大10ms待つ。
 * 0: アイドル確認  -ETIMEDOUT: タイムアウト
 */
static int pt1_wait_dma_idle(PT1_DEVICE *dev)
{
        int i;

        for (i = 0; i < 1000; i++) {
                if (!(readl(dev->regs) & DMA_BUSY))
                        return 0;
                udelay(10);
        }
        pr_err("pt1: pt1_wait_dma_idle: timeout\n");
        return -ETIMEDOUT;
}

/*
 * pt1_dma_recover - DMA stall / descriptor 破損から復旧する。
 * stop → idle 確認 → reset → restart の順で実行。
 */
static int pt1_dma_recover(PT1_DEVICE *dev, enum pt1_recover_reason reason)
{
        static const char * const reason_str[] = {
                "stall", "sync_loss", "descriptor", "timeout"
        };
        int rc = 0;

        /* permanent fault: これ以上 recover しない */
        if (smp_load_acquire(&dev->device_dead))
                return -EIO;

        /*
         * single-flight guard: watchdog と DMA thread が同時に recover に入るのを防ぐ。
         * 先着 1 つだけ実行し、後続は静かにスキップ。
         */
        if (atomic_xchg(&dev->recovering, 1))
                return 0;

        /* backoff 期限中はスキップ（reset storm を防ぐ） */
        if (time_before(jiffies, READ_ONCE(dev->recover_backoff_until))) {
                rc = 0;
                goto out;
        }

        atomic64_inc(&dev->recover_count);

        /* reason 別カウンタ */
        switch (reason) {
        case PT1_RECOVER_STALL:       atomic64_inc(&dev->recover_stall);      break;
        case PT1_RECOVER_SYNC_LOSS:   atomic64_inc(&dev->recover_sync_loss);  break;
        case PT1_RECOVER_DESCRIPTOR:  atomic64_inc(&dev->recover_descriptor); break;
        case PT1_RECOVER_TIMEOUT:     atomic64_inc(&dev->recover_timeout);    break;
        }

        pr_warn_ratelimited("pt1: card%d dma_recover reason=%s counter=%lld recover=%lld streak=%d\n",
                     dev->card_number,
                     reason_str[reason],
                     atomic64_read(&dev->dma_counter),
                     atomic64_read(&dev->recover_count),
                     dev->recover_fail_streak);

        /* recover 開始: 全 channel の consumer に recover 中を通知 */
        {
                int chi;
                for (chi = 0; chi < MAX_CHANNEL; chi++) {
                        if (dev->channel[chi])
                                smp_store_release(&dev->channel[chi]->ch_recovering, true);
                }
        }

        pt1_dma_stop(dev);

        if (pt1_wait_dma_idle(dev)) {
                pr_err("pt1: DMA did not go idle, forcing reset\n");
                atomic64_inc(&dev->recover_timeout);
                rc = -ETIMEDOUT;
                /* timeout は失敗扱いにして streak をインクリメント */
                goto update_streak;
        }

        reset_dma(dev);
        rc = 0;

update_streak:
        /* recover 完了: consumer への通知を解除 */
        {
                int chi;
                for (chi = 0; chi < MAX_CHANNEL; chi++) {
                        if (dev->channel[chi])
                                smp_store_release(&dev->channel[chi]->ch_recovering, false);
                }
        }
        if (rc == 0) {
                /* 成功: streak リセット、backoff 解除 */
                dev->recover_fail_streak = 0;
                WRITE_ONCE(dev->recover_backoff_until, jiffies);
        } else {
                /*
                 * 失敗: 指数 backoff（1s → 2s → 4s → ... → 最大 64s）
                 * streak が 10 に達したら device_dead へ遷移。
                 */
                dev->recover_fail_streak++;
                if (dev->recover_fail_streak >= 10) {
                        pr_err("pt1: card%d: %d consecutive recover failures, "
                               "marking device dead\n",
                               dev->card_number, dev->recover_fail_streak);
                        smp_store_release(&dev->device_dead, true);
                        WRITE_ONCE(dev->dma_ready, false);
                        /*
                         * 全 waitqueue を起こして poll/read が即座に
                         * EPOLLERR / -EIO を受け取れるようにする。
                         */
                        {
                                int wi;
                                for (wi = 0; wi < MAX_CHANNEL; wi++) {
                                        if (dev->channel[wi])
                                                wake_up_all(&dev->channel[wi]->wait_q);
                                }
                        }
                } else {
                        unsigned long backoff = HZ << min(dev->recover_fail_streak - 1, 6);
                        WRITE_ONCE(dev->recover_backoff_until, jiffies + backoff);
                        pr_warn("pt1: card%d: recover failed (streak=%d), "
                                "backoff %ums\n",
                                dev->card_number, dev->recover_fail_streak,
                                jiffies_to_msecs(backoff));
                }
        }

out:
        /* recovering clear: success / error / timeout / early exit すべてでここを通る */
        atomic_set(&dev->recovering, 0);
        return rc;
}

static  void    reset_dma(PT1_DEVICE *dev_conf)
{

        int             lp ;
        __u32   addr ;
        int             ring_pos = 0;
        int             data_pos = 0 ;
        __u32   *dataptr ;

        /*
         * FIX: DMA を物理的に再起動すると、それ以前に受信していたバイト列と
         * 再起動後に流れてくるバイト列は連続性が保証されない(間に欠落や
         * ギャップがある)。しかし各チャンネルの packet_size/packet_buf や
         * sync_hunting/hunt_buf はチャンネル構造体側に残ったままだったため、
         * reset_dma() を経由するたびに「リセット前の断片」と「リセット後の
         * 新しいバイト列」が同じバッファ内でつながってしまい、
         * pt1_ts_sync_recover() が両者にまたがる偽の周期一致を検出して
         * 誤ったオフセットを「同期確立」と誤認する可能性があった
         * (recover は弱電界時ほど頻発するため、まさに信号が悪い時に
         * 壊れたデータを誤って正常パケットとして書き出しかねない)。
         * reset_dma() は recover/resume 双方の唯一の共通経路なので、
         * ここで全チャンネルの組み立て状態を一括してリセットする。
         */
        for (lp = 0; lp < MAX_CHANNEL; lp++) {
                PT1_CHANNEL *ch = dev_conf->channel[lp];

                if (!ch)
                        continue;
                ch->packet_size  = 0;
                ch->sync_hunting = false;
                ch->hunt_size    = 0;
        }

        // データ初期化（sentinel word をゼロクリア）
        for(ring_pos = 0 ; ring_pos < DMA_RING_SIZE ; ring_pos++){
                for(data_pos = 0 ; data_pos < DMA_RING_MAX ; data_pos++){
                        dataptr = (dev_conf->dmactl[ring_pos])->data[data_pos];
                        dataptr[(DMA_SIZE / sizeof(__u32)) - 2] = 0;
                }
        }
        /*
         * wmb(): descriptor の書き込みが DMA engine から可視化されることを保証する。
         * descriptor は通常の CPU 書き込みであり、後続の writel(DMA_ENABLE) が
         * descriptor より先に見えると DMA がゴミデータを処理する可能性がある。
         */
        wmb();
        // 転送カウンタをリセット
        writel(0x00000010, dev_conf->regs);
        // 転送カウンタをインクリメント
        for(lp = 0 ; lp < DMA_RING_SIZE ; lp++){
                writel(0x00000020, dev_conf->regs);
        }

        /*
         * FIX: dma_addr_t は64bitになり得るため、ハードウェアへ渡す際は
         * 下位32bitのみを使用する（DMA_BIT_MASK(32)を設定しているため安全）。
         * キャストは dma_addr_t のまま扱い、writelに渡す直前に lower_32_bits() を使う。
         */
        addr = lower_32_bits(dev_conf->ring_dma[0]) >> 12;
        // DMAバッファ設定
        writel(addr, dev_conf->regs + DMA_ADDR);
        // DMA開始
        writel(0x0c000040, dev_conf->regs);
        /* posted write flush: PCI write が確実にデバイスに届いてから戻る */
        readl(dev_conf->regs + DMA_ADDR);
        mb();

}
/*
 * pt1_ring_write - circular ring buffer への1パケット書き込み (lockless fast path)
 *
 * smp_store_release(head) によって producer 側の書き込み完了を
 * consumer 側に可視化する。ring が満杯の場合は drop カウントだけ増やして捨てる。
 */
static void pt1_ring_write(PT1_CHANNEL *channel, const u8 *pkt)
{
        struct ts_ring *ring = &channel->ring;
        u32 next;

        next = (ring->head + 1) & ring_mask;
        if (unlikely(next == READ_ONCE(ring->tail))) {
                /*
                 * ring full: 最古パケット(tail)を1つ捨てて空きを作る。
                 *
                 * consumer(pt1_read) も tail を更新するため、
                 * overflow_lock で producer/consumer の tail 競合を防ぐ。
                 * 通常の fastpath（overflow なし）は lockless のまま。
                 */
                unsigned long flags;
                spin_lock_irqsave(&ring->overflow_lock, flags);
                if (next == READ_ONCE(ring->tail)) {
                        WRITE_ONCE(ring->tail,
                                (READ_ONCE(ring->tail) + 1) & ring_mask);
                        atomic_inc(&channel->drop);
                        atomic_inc(&channel->overflow_ring);
                }
                spin_unlock_irqrestore(&ring->overflow_lock, flags);
        }
        /*
         * prefetchw: 次のパケットの書き込み先 cacheline を Exclusive state で先取り。
         * prefetch（共有読み込み）より書き込み用の prefetchw の方が
         * E-core（N100 等）では効果的。
         */
        prefetchw(ring->buffer + next * PACKET_SIZE);
        memcpy(ring->buffer + ring->head * PACKET_SIZE, pkt, PACKET_SIZE);
        smp_store_release(&ring->head, next);

        /* max_fill 更新: cmpxchg で SMP race-free に最大値を記録 */
        {
                u32 used = (next - READ_ONCE(ring->tail) + ring_count) & ring_mask;
                u32 old;

                do {
                        old = READ_ONCE(channel->max_fill);
                        if (used <= old)
                                break;
                } while (cmpxchg(&channel->max_fill, old, used) != old);

                /*
                 * ring occupancy histogram: 4段階。
                 * [0]=0〜25%  [1]=25〜50%  [2]=50〜75%  [3]=75〜100%
                 * ring_count は power-of-two なので shift で除算を回避。
                 * shift = ilog2(ring_count) - 2 (実行時計算)
                 */
                atomic64_inc(&channel->fill_hist[
                        min_t(u32, used >> (ilog2(ring_count) - 2), 3)]);

                /* 75% 超えたら debug メッセージを記録（ratelimited で長時間録画でも安全） */
                if (unlikely(used * 4 >= ring_count * 3))
                        pr_debug_ratelimited("pt1: ring high ch=%u fill=%u/%u\n",
                                             channel->channel, used, ring_count);
        }
}

/*
 * pt1_channel_wakeup - consumer を起こす。
 * WAKEUP_WATERMARK 以上 OR 最後の wake から 100ms 経過（starvation 防止）
 * 低ビットレート ch でも最低 10Hz で consumer が起きられる。
 * 元は pt1_ring_write 呼び出し後にインラインで書かれていたが、
 * TS sync hunting からも複数パケットまとめて書き込んだ後に
 * 同じ処理を呼びたいため、共通関数に切り出した。
 */
static void pt1_channel_wakeup(PT1_CHANNEL *channel)
{
        u32 head = READ_ONCE(channel->ring.head);
        u32 tail = READ_ONCE(channel->ring.tail);
        u32 avail = (head - tail + ring_count) & ring_mask;

        if (avail >= WAKEUP_WATERMARK ||
            (avail > 0 && time_after(jiffies,
                    READ_ONCE(channel->last_wake_jiffies) + HZ/10))) {
                WRITE_ONCE(channel->last_wake_jiffies, jiffies);
                atomic64_inc(&channel->wakeup_count);
                wake_up_interruptible_poll(&channel->wait_q,
                                            EPOLLIN | EPOLLRDNORM);
        }
}

/*
 * pt1_ts_sync_recover - buf の中から「188バイト間隔で3連続0x47」となる
 * 位置(TS パケット境界の候補)を探して返す。見つからなければ -1。
 *
 * 本来のTS再同期に必要な検証で、377バイト以上の連続データが必要
 * (buf[i], buf[i+188], buf[i+376] の3点を見る)。呼び出し側は
 * hunt_buf(3パケット分, 564byte)を渡すことでこの前提を満たす。
 *
 * NOTE: 以前は呼び出し側が1パケット分(188〜190byte)しか渡しておらず、
 * len - 376 が常に負になってこの関数が無条件に -1 を返し続けていた
 * (再同期機能が実質丸ごと無効化されていた)。呼び出し側を
 * hunting方式(pt1_thread の sync_hunting 状態)に変更し、
 * 十分な長さのバッファを渡すよう修正した。
 */
static int pt1_ts_sync_recover(const u8 *buf, int len)
{
        int i;

        for (i = 0; i < len - 376; i++) {
                if (buf[i] == 0x47 &&
                    buf[i + 188] == 0x47 &&
                    buf[i + 376] == 0x47)
                        return i;
        }
        return -1;
}

static  int             pt1_thread(void *data)
{
        PT1_DEVICE      *dev_conf = data ;
        PT1_CHANNEL     *channel ;
        int             ring_pos = 0;
        int             data_pos = 0 ;
        int             lp ;
        int             chno ;
        int             dma_channel ;
        int             packet_pos ;
        __u32   *dataptr ;
        __u32   *curdataptr ;
        __u32   val ;
        bool    dma_recovered_this_buf ;
        union   mpacket{
                __u32   val ;
                MICRO_PACKET    packet ;
        }micro;

        set_freezable();
        /*
         * DMA thread のスケジューリング優先度。
         * dma_rt=1 の場合のみ SCHED_FIFO(low) に昇格する。
         * デフォルト OFF: N100 の 4E-core 環境では RT thread が
         * NVMe flush / softirq を詰まらせ逆効果になる場合がある。
         * CPU pinning(dma_cpu) だけでも十分なことが多い。
         *
         * FIX: sched_set_fifo_low() は Linux 5.9 で追加された API。
         * 対象カーネルが 6.8(Ubuntu 24.04) 以降のみになったため
         * LINUX_VERSION_CODE 分岐は不要（常に真）。削除して単純化。
         */
        if (dma_rt)
                sched_set_fifo_low(current);
        /*
         * DMA thread 起動時に reset_dma を呼んで DMA を開始する。
         * pt1_pci_init_one で pt1_dma_init / pt1_makering を実行済みだが、
         * DMA の実際の開始（レジスタへの書き込み）は reset_dma が行う。
         */
        reset_dma(dev_conf);
        printk(KERN_INFO "pt1_thread run\n");

        for(;;){
                if(kthread_should_stop()){
                        break ;
                }

                for(;;){
                        dataptr = (dev_conf->dmactl[ring_pos])->data[data_pos];

                        /* --- descriptor validity check --- */
                        if (unlikely(dataptr == NULL)) {
                                pr_err("pt1: NULL descriptor at ring=%d data=%d\n",
                                       ring_pos, data_pos);
                                channel = dev_conf->channel[0];
                                if (channel) atomic_inc(&channel->drop);
                                if (time_after(jiffies, READ_ONCE(dev_conf->last_reset) + HZ)) {
                                        WRITE_ONCE(dev_conf->last_reset, jiffies);
                                        pt1_dma_recover(dev_conf, PT1_RECOVER_DESCRIPTOR);
                                }
                                ring_pos = data_pos = 0;
                                break;
                        }

                        // データあり？
                        if(dataptr[(DMA_SIZE / sizeof(__u32)) - 2] == 0){
                                /*
                                 * データなし: DMA thread は正常に polling しているが
                                 * ハードウェアからデータが来ていない（チューナ未使用 / 信号なし）。
                                 * last_dma_jiffies を更新して watchdog の誤 stall 判定を防ぐ。
                                 */
                                WRITE_ONCE(dev_conf->last_dma_jiffies, jiffies);
                                break ;
                        }

                        /* watchdog: DMA が動いていることを記録 */
                        atomic64_inc(&dev_conf->dma_counter);
                        WRITE_ONCE(dev_conf->last_dma_jiffies, jiffies);

                        micro.val = *dataptr ;
                        curdataptr = dataptr ;
                        data_pos += 1 ;
                        dma_recovered_this_buf = false;
                        for(lp = 0 ; lp < (DMA_SIZE / sizeof(__u32)) ; lp++, dataptr++){
                                micro.val = *dataptr ;
                                dma_channel = ((micro.packet.head >> 5) & 0x07);
                                //チャネル情報不正
                                /* dma_channel は 1〜4 が有効。0 はパディング、5以上は不正 */
                                if(dma_channel == 0 || dma_channel > MAX_CHANNEL){
                                        if(dma_channel != 0)
                                                printk(KERN_ERR "DMA Channel Number Error(%d)\n", dma_channel);
                                        continue ;
                                }
                                chno = real_channel[dma_channel - 1];
                                packet_pos = ((micro.packet.head >> 2) & 0x07);
                                if (unlikely(chno >= MAX_CHANNEL || dev_conf->channel[chno] == NULL)) {
                                        pr_err_ratelimited("pt1: invalid chno=%d dma_channel=%d\n",
                                                           chno, dma_channel);
                                        continue;
                                }
                                channel = dev_conf->channel[chno] ;
                                //  エラーチェック
                                if((micro.packet.head & MICROPACKET_ERROR)){
                                        val = readl(dev_conf->regs);
                                        if((val & BIT_RAM_OVERFLOW)){
                                                atomic_inc(&channel->overflow);
                                        }
                                        if((val & BIT_INITIATOR_ERROR)){
                                                WRITE_ONCE(channel->counetererr, READ_ONCE(channel->counetererr) + 1);
                                        }
                                        if((val & BIT_INITIATOR_WARNING)){
                                                WRITE_ONCE(channel->transerr, READ_ONCE(channel->transerr) + 1);
                                        }
                                        // 初期化して先頭から
                                        if (time_after(jiffies, READ_ONCE(dev_conf->last_reset) + HZ)) {
                                                WRITE_ONCE(dev_conf->last_reset, jiffies);
                                                pt1_dma_recover(dev_conf, PT1_RECOVER_SYNC_LOSS);
                                                /*
                                                 * FIX: pt1_dma_recover() は内部で reset_dma() を
                                                 * 呼び、全バッファのセンチネルを既にゼロクリアし、
                                                 * DMA も再起動済み。この後さらに curdataptr(古い
                                                 * ring_pos/data_pos 時点のポインタ)へ書き込むのは
                                                 * 冗長なだけでなく、再起動した DMA ハードウェアが
                                                 * 同じバッファへ書き込み始めるタイミングと衝突する
                                                 * レースになり得るため、この場合だけスキップする。
                                                 */
                                                dma_recovered_this_buf = true;
                                        }
                                        ring_pos = data_pos = 0 ;
                                        break ;
                                }
                                // 未使用チャネルは捨てる
                                if (!READ_ONCE(channel->valid)) {
                                        continue ;
                                }

                                /*
                                 * TS sync hunting 中: 通常のパケット単位処理は行わず、
                                 * 複数パケット分(3パケット=564byte)のスクラッチバッファに
                                 * 生バイト列を貯めて、本来の周期チェック(188バイト間隔で
                                 * 3連続0x47)による再同期を試みる。
                                 * ハードウェアの「先頭フラグ」(head & 0x02)は同期が
                                 * 崩れている間は信用できないため、ここでは無視して
                                 * 連続的にバイトを積み上げる。
                                 */
                                if (unlikely(channel->sync_hunting)) {
                                        channel->hunt_buf[channel->hunt_size]   = micro.packet.data[2];
                                        channel->hunt_buf[channel->hunt_size+1] = micro.packet.data[1];
                                        channel->hunt_buf[channel->hunt_size+2] = micro.packet.data[0];
                                        channel->hunt_size += 3;

                                        if (channel->hunt_size >= sizeof(channel->hunt_buf)) {
                                                int off = pt1_ts_sync_recover(
                                                        channel->hunt_buf, channel->hunt_size);

                                                if (off >= 0) {
                                                        u32 avail  = channel->hunt_size - off;
                                                        u32 nfull  = avail / PACKET_SIZE;
                                                        u32 remain = avail - nfull * PACKET_SIZE;
                                                        u32 k;

                                                        /*
                                                         * off, off+188, off+376 の3点が確認できた
                                                         * ことで周期性(=正しい境界)が確認できている。
                                                         * off から数えて完全に収まっている分だけ
                                                         * まとめて書き出す。
                                                         */
                                                        for (k = 0; k < nfull; k++)
                                                                pt1_ring_write(channel,
                                                                        channel->hunt_buf + off + k * PACKET_SIZE);

                                                        if (remain) {
                                                                memcpy(channel->packet_buf,
                                                                       channel->hunt_buf + off + nfull * PACKET_SIZE,
                                                                       remain);
                                                        }
                                                        channel->packet_size = remain;
                                                        channel->sync_hunting = false;

                                                        pr_info_ratelimited("pt1: ch=%d TS re-sync OK (off=%d, %u packets recovered)\n",
                                                                            channel->channel, off, nfull);
                                                        if (nfull)
                                                                pt1_channel_wakeup(channel);
                                                } else {
                                                        /* このウィンドウでは見つからず。捨てて次のウィンドウで再挑戦する */
                                                        atomic_inc(&channel->drop);
                                                }
                                                channel->hunt_size = 0;
                                        }
                                        continue;
                                }

                                // 先頭で、一時バッファに残っている場合
                                if((micro.packet.head & 0x02) &&  (channel->packet_size != 0)){
                                        channel->packet_size = 0 ;
                                }
                                // データコピー
                                channel->packet_buf[channel->packet_size]   = micro.packet.data[2];
                                channel->packet_buf[channel->packet_size+1] = micro.packet.data[1];
                                channel->packet_buf[channel->packet_size+2] = micro.packet.data[0];
                                channel->packet_size += 3;

                                // パケットが完成したら circular ring buffer へ書き込む
                                if(channel->packet_size >= PACKET_SIZE){
                                        if (unlikely(channel->packet_buf[0] != 0x47)) {
                                                /*
                                                 * sync loss: 1パケット分のバッファだけでは
                                                 * 正しい境界を判定できないため、hunting
                                                 * モードに切り替えて複数パケット分の
                                                 * データで再同期を試みる。
                                                 * このパケット自体の内容は信頼できないため破棄する。
                                                 */
                                                atomic64_inc(&dev_conf->sync_loss);
                                                pr_debug_ratelimited("pt1: sync loss ch=%d sync_loss=%lld drop=%d, entering hunt\n",
                                                             channel->channel,
                                                             atomic64_read(&dev_conf->sync_loss),
                                                             atomic_read(&channel->drop));
                                                atomic_inc(&channel->drop);
                                                channel->packet_size = 0;
                                                channel->sync_hunting = true;
                                                channel->hunt_size = 0;
                                                continue;
                                        }

                                        /* lockless write to circular ring buffer */
                                        pt1_ring_write(channel, channel->packet_buf);
                                        channel->packet_size = 0 ;
                                        pt1_channel_wakeup(channel);
                                }
                        }
                        /*
                         * FIX: MICROPACKET_ERROR で pt1_dma_recover() が
                         * 実行された場合、reset_dma() が既に全バッファの
                         * センチネルをゼロクリアし DMA も再起動済みなので、
                         * ここで stale な curdataptr へ書き込むのは冗長かつ
                         * 再起動直後の DMA と衝突するレースになり得る。
                         * その場合はスキップする。
                         */
                        if (!dma_recovered_this_buf)
                                curdataptr[(DMA_SIZE / sizeof(__u32)) - 2] = 0;

                        if(data_pos >= DMA_RING_MAX){
                                data_pos = 0;
                                ring_pos += 1 ;
                                // DMAリングが変わった場合はインクリメント
                                writel(0x00000020, dev_conf->regs);
                                if(ring_pos >= DMA_RING_SIZE){
                                        ring_pos = 0 ;
                                }
                        }

                        /*
                         * FIX: この内側ループは「データがある限り」回り続ける設計で、
                         * ループ内に一度も CPU 譲渡ポイント(cond_resched)や
                         * kthread_should_stop() チェックがなかった。信号が強く
                         * データが途切れなく流れ続ける状況(あるいは何らかの異常で
                         * センチネル判定が壊れた場合)、理論上このループが
                         * 際限なく回り続け、rmmod 時に kthread_stop() が
                         * 反応しない、あるいは他タスクのスケジューリングに
                         * 悪影響を与える可能性がある。実害が確認できているわけ
                         * ではないが、コストがほぼゼロなので保険として入れる。
                         */
                        if (unlikely(kthread_should_stop()))
                                goto out;
                        cond_resched();
                }
                /*
                 * polling interval: 10〜15ms。
                 * ring が 131072 slots と大きいため wakeup 回数より
                 * 短め sleep + watermark batching の方が安定する。
                 * usleep_range は hrtimer ベースで HZ=250 環境（N100 等）でも
                 * msleep(20) のような 24〜28ms への膨張が起きない。
                 */
                try_to_freeze();
                usleep_range(10000, 15000);
        }
out:
        return 0 ;
}

/*
 * pt1_watchdog - DMA stall 監視スレッド。
 * dma_counter が HZ*3 間進まなければ DMA recover を実行する。
 *
 * 判定ロジック:
 *   毎周回(500ms)で counter を読み、前回値と比較する。
 *   time_after(last_dma_jiffies + HZ*3) は DMA thread が
 *   最後にパケットを処理した時刻からの経過時間で判断する。
 *   recover 後は HZ*5 のクールダウンを設けて false stall を防ぐ。
 */
static int pt1_watchdog(void *data)
{
        PT1_DEVICE *dev_conf = data;
        s64 prev = 0;
        unsigned long cooldown_until = 0;

        set_freezable();
        while (!kthread_should_stop()) {
                msleep(500);

                /* systemd suspend / hibernate との協調 */
                try_to_freeze();

                if (kthread_should_stop())
                        break;

                /*
                 * suspend 中 / DMA 未初期化時はスキップ。
                 * resume 完了後に dma_ready が true になってから監視を再開する。
                 * device_dead の場合は永続障害なので watchdog も停止。
                 */
                if (!smp_load_acquire(&dev_conf->dma_ready) || smp_load_acquire(&dev_conf->device_dead))
                        continue;

                /* recover 直後のクールダウン期間はスキップ */
                if (time_before(jiffies, cooldown_until)) {
                        prev = atomic64_read(&dev_conf->dma_counter);
                        continue;
                }

                {
                        s64 cur = atomic64_read(&dev_conf->dma_counter);

                        if (cur != prev) {
                                /* DMA が進んでいる: last_dma_jiffies は DMA thread が更新する */
                                prev = cur;
                                continue;
                        }

                        /*
                         * counter が前回と同じ: DMA thread がパケットを処理していない。
                         * last_dma_jiffies から HZ*10 経過していれば stall と判定する。
                         * DMA thread はパケットがあるときもなくても last_dma_jiffies を更新するため、
                         * 「DMA は動いているがデータが来ていない」は正確に除外できる。
                         * 閾値を 10秒にすることで aggressive な recover を抑制し、
                         * 本当に DMA が死んだときだけ recover する。
                         */
                        if (!time_after(jiffies, READ_ONCE(dev_conf->last_dma_jiffies) + HZ * 10))
                                continue;

                        pr_err_ratelimited("pt1: DMA stall detected (counter=%lld), recovering\n", cur);
                        pt1_dma_recover(dev_conf, PT1_RECOVER_STALL);

                        /* recover 後 5秒のクールダウン: DMA 再起動が安定するまで待つ */
                        cooldown_until = jiffies + HZ * 5;
                        WRITE_ONCE(dev_conf->last_dma_jiffies, jiffies);
                        prev = atomic64_read(&dev_conf->dma_counter);
                }
        }
        return 0;
}
static int pt1_open(struct inode *inode, struct file *file)
{
        int             major = imajor(inode);
        int             minor = iminor(inode);
        int             lp ;
        int             lp2 ;
        PT1_CHANNEL     *channel ;

        for(lp = 0 ; lp < MAX_PCI_DEVICE ; lp++){
                if(device[lp] == NULL){
                        return -EIO ;
                }

                if(MAJOR(device[lp]->dev) == major &&
                   device[lp]->base_minor <= minor &&
                   device[lp]->base_minor + MAX_CHANNEL > minor) {

                        /* device_dead: 永続的障害。open から -EIO を返して userspace に通知 */
                        if (smp_load_acquire(&device[lp]->device_dead)) {
                                pr_warn("pt1: card%d: open rejected, device is dead\n",
                                        device[lp]->card_number);
                                return -EIO;
                        }

                        mutex_lock(&device[lp]->lock);
                        for(lp2 = 0 ; lp2 < MAX_CHANNEL ; lp2++){
                                channel = device[lp]->channel[lp2] ;
                                if(channel->minor == minor){
                                        if(channel->valid == TRUE){
                                                mutex_unlock(&device[lp]->lock);
                                                return -EIO ;
                                        }

                                        /* wake tuner up */
                                        set_sleepmode(channel->ptr->regs, &channel->lock,
                                                                  channel->address, channel->type,
                                                                  TYPE_WAKEUP);
                                        msleep(100);

                                        atomic_set(&channel->drop, 0);
                                        WRITE_ONCE(channel->valid, TRUE);
                                        atomic_set(&channel->overflow, 0);
                                        WRITE_ONCE(channel->counetererr, 0);
                                        WRITE_ONCE(channel->transerr, 0);
                                        channel->packet_size = 0 ;
                                        /*
                                         * FIX: close中にたまたま sync_hunting 中だった場合、
                                         * hunt_buf に前セッションの古い断片データが残った
                                         * ままになる。reset_dma() と同じ理由で、再オープン時
                                         * にもここでリセットしないと、再開後の新しいバイト列と
                                         * 継ぎ足されて偽の同期一致を誤検出しかねない。
                                         */
                                        channel->sync_hunting = false;
                                        channel->hunt_size = 0;
                                        file->private_data = channel;
                                        /* ring buffer リセット (open 時に head/tail を揃える) */
                                        WRITE_ONCE(channel->ring.head, 0);
                                        WRITE_ONCE(channel->ring.tail, 0);
                                        mutex_unlock(&device[lp]->lock);
                                        return 0 ;
                                }
                        }
                }
        }
        return -EIO;
}
static int pt1_release(struct inode *inode, struct file *file)
{
        PT1_CHANNEL     *channel = file->private_data;

        mutex_lock(&channel->ptr->lock);
        SetStream(channel->ptr->regs, channel->channel, FALSE);
        WRITE_ONCE(channel->valid, FALSE);
        printk(KERN_INFO "(%d:%d)Drop=%08d:%08d:%08d:%08d\n", imajor(inode), iminor(inode), atomic_read(&channel->drop),
                                                atomic_read(&channel->overflow), channel->counetererr, channel->transerr);
        atomic_set(&channel->overflow, 0);
        WRITE_ONCE(channel->counetererr, 0);
        WRITE_ONCE(channel->transerr, 0);
        atomic_set(&channel->drop, 0);
        mutex_unlock(&channel->ptr->lock);

        /* send tuner to sleep */
        set_sleepmode(channel->ptr->regs, &channel->lock,
                                  channel->address, channel->type, TYPE_SLEEP);
        msleep(100);

        return 0;
}

static ssize_t pt1_read(struct file *file, char __user *buf, size_t cnt, loff_t * ppos)
{
        PT1_CHANNEL     *channel = file->private_data;
        struct ts_ring  *ring = &channel->ring;
        u32             head, avail, to_copy;
        ssize_t         copied = 0;

        /* device_dead: 永続的障害。read から -EIO を返して recisdb 等に通知 */
        if (unlikely(smp_load_acquire(&channel->ptr->device_dead)))
                return -EIO;

        /*
         * ch_recovering: DMA recover 中は ring の head/tail が不定になり得る。
         * -EAGAIN を返して userspace に retry を促す。
         * O_NONBLOCK でなくても recover は短時間（通常 < 100ms）なので
         * blocking read でも次の呼び出しで正常データが取れる。
         */
        if (unlikely(smp_load_acquire(&channel->ch_recovering)))
                return -EAGAIN;

        /*
         * cnt を 188 byte 単位に切り捨てる。
         * partial packet を返すと userspace 側で TS alignment が崩れる。
         */
        cnt -= cnt % PACKET_SIZE;
        if (cnt == 0)
                return -EINVAL;

        if (file->f_flags & O_NONBLOCK) {
                /*
                 * O_NONBLOCK: ring に PACKET_SIZE 分なければ即 EAGAIN。
                 * 不要な wait / mutex を一切踏まない。
                 */
                head = smp_load_acquire(&ring->head);
                avail = (head - READ_ONCE(ring->tail) + ring_count) & ring_mask;
                if (avail == 0) {
                        atomic_inc(&channel->underrun_count);
                        return -EAGAIN;
                }
        } else {
                /*
                 * blocking: WAKEUP_WATERMARK 分溜まるまで待つ。
                 * poll/epoll も wait_queue を共有するので
                 * wake_up_interruptible_poll と対になっている。
                 */
                wait_event_interruptible_timeout(channel->wait_q,
                        ({
                                head = smp_load_acquire(&ring->head);
                                ((head - READ_ONCE(ring->tail) + ring_count)
                                        & ring_mask) >= WAKEUP_WATERMARK;
                        }) || signal_pending(current),
                        msecs_to_jiffies(500));
        }

        head  = smp_load_acquire(&ring->head);
        avail = (head - READ_ONCE(ring->tail) + ring_count) & ring_mask;

        if (avail == 0) {
                atomic_inc(&channel->underrun_count);
                return 0;
        }

        /*
         * bulk copy: ring の contiguous な部分をまとめてコピーする。
         * tail から head まで、または ring 末端まで、どちらか短い方を
         * 一度の copy_to_user で転送する。cache miss と syscall overhead を削減。
         */
        to_copy = min_t(u32, avail, (u32)(cnt / PACKET_SIZE));

        while (to_copy > 0) {
                u32 tail    = READ_ONCE(ring->tail);
                /* ring 末端までの連続スロット数 */
                u32 contig  = min_t(u32, to_copy, ring_count - tail);
                size_t bytes = (size_t)contig * PACKET_SIZE;
                const u8 *src = ring->buffer + tail * PACKET_SIZE;

                /* prefetch: 次の contiguous ブロック先頭をキャッシュに乗せる */
                prefetch(ring->buffer + ((tail + contig) & ring_mask) * PACKET_SIZE);

                if (copy_to_user(buf + copied, src, bytes))
                        return copied ? (ssize_t)copied : -EFAULT;

                WRITE_ONCE(ring->tail, (tail + contig) & ring_mask);
                copied  += bytes;
                to_copy -= contig;
        }

        return copied;
}
static  int             SetFreq(PT1_CHANNEL *channel, FREQUENCY *freq)
{

        switch(channel->type){
                case CHANNEL_TYPE_ISDB_S:
                        {
                                ISDB_S_TMCC             tmcc ;
                                if(bs_tune(channel->ptr->regs,
                                                &channel->ptr->lock,
                                                channel->address,
                                                freq->frequencyno,
                                                &tmcc) < 0){
                                        return -EIO ;
                                }
                                ts_lock(channel->ptr->regs,
                                                &channel->ptr->lock,
                                                channel->address,
                                                tmcc.ts_id[freq->slot].ts_id);
                        }
                        break ;
                case CHANNEL_TYPE_ISDB_T:
                        {
                                if(isdb_t_frequency(channel->ptr->regs,
                                                &channel->ptr->lock,
                                                channel->address,
                                                freq->frequencyno, freq->slot) < 0){
                                        return -EINVAL ;
                                }
                        }
        }
        return 0 ;
}

static int count_used_bs_tuners(PT1_DEVICE *device)
{
        int count = 0;
        int i;

        for(i=0; i<MAX_CHANNEL; i++) {
                if(device && device->channel[i] &&
                   device->channel[i]->type == CHANNEL_TYPE_ISDB_S &&
                   device->channel[i]->valid)
                        count++;
        }

        printk(KERN_INFO "used bs tuners on %p = %d\n", device, count);
        return count;
}

static long pt1_do_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
        PT1_CHANNEL     *channel = file->private_data;
        int             signal = 0;
        unsigned long   dummy;
        void            *arg = (void *)arg0;
        int             lnb_eff, lnb_usr;
        char *voltage[] = {"0V", "11V", "15V"};
        int count;

        /* device_dead: 操作不可 */
        if (unlikely(smp_load_acquire(&channel->ptr->device_dead)))
                return -EIO;

        switch(cmd){
                case SET_CHANNEL:
                        {
                                FREQUENCY       freq ;
                                dummy = copy_from_user(&freq, arg, sizeof(FREQUENCY));
                                if(dummy) {
                                	return -EFAULT;
                                }
                                return SetFreq(channel, &freq);
                        }
                case START_REC:
                        /*
                         * FIX: SetStream() は dev_conf->regs (TS_TEST_ENABLE_ADDR=0x08)
                         * に書き込むが、この ioctl 経路では channel->lock (チャンネル単位)
                         * しか保護しておらず、i2c_read()/i2c_write() が使う
                         * channel->ptr->lock (デバイス単位、dev_conf->regs 全体を保護)
                         * を取っていなかった。同じカードの別チャンネルが同時に
                         * I2C操作(SET_CHANNEL/GET_SIGNAL_STRENGTH等)を行うと、
                         * dev_conf->regs への書き込みと読み出しがロックなしで競合し得る
                         * (I2C_RESULT_ADDR と TS_TEST_ENABLE_ADDR は同一アドレス 0x08)。
                         * pt1_release() と同じく channel->ptr->lock で保護する。
                         */
                        mutex_lock(&channel->ptr->lock);
                        SetStream(channel->ptr->regs, channel->channel, TRUE);
                        mutex_unlock(&channel->ptr->lock);
                        return 0 ;
                case STOP_REC:
                        mutex_lock(&channel->ptr->lock);
                        SetStream(channel->ptr->regs, channel->channel, FALSE);
                        mutex_unlock(&channel->ptr->lock);
                        msleep(100);
                        return 0 ;
                case GET_SIGNAL_STRENGTH:
                        switch(channel->type){
                                case CHANNEL_TYPE_ISDB_S:
                                        signal = isdb_s_read_signal_strength(channel->ptr->regs,
                                                                                                &channel->ptr->lock,
                                                                                                channel->address);
                                        break ;
                                case CHANNEL_TYPE_ISDB_T:
                                        signal = isdb_t_read_signal_strength(channel->ptr->regs,
                                                                                                &channel->ptr->lock, channel->address);
                                        break ;
                        }
                        dummy = copy_to_user(arg, &signal, sizeof(int));
                        if (dummy) {
                                return -EFAULT;
                        }
                        return 0 ;
                case LNB_ENABLE:
                        count = count_used_bs_tuners(channel->ptr);
                        if(count <= 1) {
                                lnb_usr = (int)arg0;
                                lnb_eff = lnb_usr ? lnb_usr : lnb;
                                /*
                                 * FIX: lnb_eff (ユーザー指定値) の範囲チェックが無く、
                                 * voltage[] (要素数3, index 0..2) を範囲外添字で
                                 * 読んでしまう可能性があった。不正な値が渡されると
                                 * printk("%s", voltage[lnb_eff]) が任意のメモリを
                                 * 文字列として読み、kernel oops を起こし得る。
                                 */
                                if (lnb_eff < LNB_OFF || lnb_eff > LNB_15V) {
                                        printk(KERN_ERR "PT1:LNB_ENABLE invalid level(%d)\n", lnb_eff);
                                        return -EINVAL;
                                }
                                settuner_reset(channel->ptr->regs, channel->ptr->cardtype, lnb_eff, TUNER_POWER_ON_RESET_DISABLE);
                                printk(KERN_INFO "PT1:LNB on %s\n", voltage[lnb_eff]);
                        }
                        return 0 ;
                case LNB_DISABLE:
                        count = count_used_bs_tuners(channel->ptr);
                        if(count <= 1) {
                                settuner_reset(channel->ptr->regs, channel->ptr->cardtype, LNB_OFF, TUNER_POWER_ON_RESET_DISABLE);
                                printk(KERN_INFO "PT1:LNB off\n");
                        }
                        return 0 ;
        }
        return -EINVAL;
}

static long pt1_unlocked_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
        PT1_CHANNEL     *channel = file->private_data;
        long ret;

        mutex_lock(&channel->lock);
        ret = pt1_do_ioctl(file, cmd, arg0);
        mutex_unlock(&channel->lock);

        return ret;
}

static long pt1_compat_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
        long ret;
        /* should do 32bit <-> 64bit conversion here? --yaz */
        ret = pt1_unlocked_ioctl(file, cmd, arg0);

        return ret;
}

static __poll_t pt1_poll(struct file *file, poll_table *wait)
{
        PT1_CHANNEL     *channel = file->private_data;
        struct ts_ring  *ring = &channel->ring;
        u32             head, avail;

        poll_wait(file, &channel->wait_q, wait);

        /* device_dead: EPOLLERR | EPOLLHUP で userspace に通知 */
        if (unlikely(smp_load_acquire(&channel->ptr->device_dead)))
                return EPOLLERR | EPOLLHUP;

        /* ch_recovering: recover 中は readable を返さない（busy-loop 防止） */
        if (unlikely(smp_load_acquire(&channel->ch_recovering)))
                return 0;

        head  = smp_load_acquire(&ring->head);
        avail = (head - READ_ONCE(ring->tail) + ring_count) & ring_mask;

        if (avail >= WAKEUP_WATERMARK)
                return EPOLLIN | EPOLLRDNORM;

        return 0;
}

/*
*/
static const struct file_operations pt1_fops = {
        .owner          =       THIS_MODULE,
        .open           =       pt1_open,
        .release        =       pt1_release,
        .read           =       pt1_read,
        .poll           =       pt1_poll,
        .unlocked_ioctl =       pt1_unlocked_ioctl,
        .compat_ioctl   =       pt1_compat_ioctl,
        .llseek         =       noop_llseek,
};

static int      pt1_makering(struct pci_dev *pdev, PT1_DEVICE *dev_conf)
{
        int             lp ;
        int             lp2 ;
        DMA_CONTROL             *dmactl;
        __u32   *dmaptr ;
        __u32   addr  ;
        __u32   *ptr ;

        //DMAリング作成
        for(lp = 0 ; lp < DMA_RING_SIZE ; lp++){
                ptr = dev_conf->dmaptr[lp];
                if(lp ==  (DMA_RING_SIZE - 1)){
                        /*
                         * FIX: dma_addr_t を直接 __u32 にキャストするのは64bit環境で
                         * アドレスが切り詰められ破壊的なバグになる。
                         * lower_32_bits() マクロで安全に下位32bitを取り出す。
                         * (DMA_BIT_MASK(32)設定済みなので物理的に32bit以内に収まる)
                         */
                        addr = lower_32_bits(dev_conf->ring_dma[0]);
                }else{
                        addr = lower_32_bits(dev_conf->ring_dma[(lp + 1)]);
                }
                addr >>= 12 ;
                memcpy(ptr, &addr, sizeof(__u32));
                ptr += 1 ;

                dmactl = dev_conf->dmactl[lp];
                for(lp2 = 0 ; lp2 < DMA_RING_MAX ; lp2++){
                        dmaptr = dma_alloc_coherent(&pdev->dev, DMA_SIZE, &dmactl->ring_dma[lp2], GFP_KERNEL);
                        if(dmaptr == NULL){
                                printk(KERN_INFO "PT1:DMA ALLOC ERROR\n");
                                /*
                                 * 確保済み分を解放: lp2 未満の内側 + lp 未満の外側。
                                 * pt1_dma_free は data[i] != NULL チェック済みなので
                                 * 呼び出し元が pt1_dma_free を呼べば全体を unwind できる。
                                 * ここでは現 ring の lp2 未満のみ解放し、残りは呼び出し元に委ねる。
                                 */
                                while (--lp2 >= 0)
                                        dma_free_coherent(&pdev->dev, DMA_SIZE,
                                                          dmactl->data[lp2],
                                                          dmactl->ring_dma[lp2]);
                                return -ENOMEM;
                        }
                        dmactl->data[lp2] = dmaptr ;
                        // DMAデータエリア初期化
                        dmaptr[(DMA_SIZE / sizeof(__u32)) - 2] = 0 ;
                        addr = lower_32_bits(dmactl->ring_dma[lp2]);
                        addr >>= 12 ;
                        memcpy(ptr, &addr, sizeof(__u32));
                        ptr += 1 ;
                }
        }
        return 0 ;
}

/* ------------------------------------------------------------------ */
/* debugfs stats                                                        */
/* ------------------------------------------------------------------ */

/*
 * ring_fill_read - channel N の ring 使用率をパーセントで返す。
 * debugfs ファイルから cat で読める。
 */
static int pt1_ring_fill_show(struct seq_file *m, void *v)
{
        PT1_CHANNEL *ch = m->private;
        u32 head = smp_load_acquire(&ch->ring.head);
        u32 tail = READ_ONCE(ch->ring.tail);
        u32 used = (head - tail + ring_count) & ring_mask;
        u32 pct  = used * 100 / ring_count;

        seq_printf(m, "%u/%u (%u%%)\n", used, ring_count, pct);
        return 0;
}
DEFINE_SHOW_ATTRIBUTE(pt1_ring_fill);

static int pt1_stats_show(struct seq_file *m, void *v)
{
        PT1_DEVICE *dev = m->private;
        int i;

        seq_printf(m, "recover_count : %lld\n",
                   atomic64_read(&dev->recover_count));
        seq_printf(m, "  stall       : %lld\n",
                   atomic64_read(&dev->recover_stall));
        seq_printf(m, "  sync_loss   : %lld\n",
                   atomic64_read(&dev->recover_sync_loss));
        seq_printf(m, "  descriptor  : %lld\n",
                   atomic64_read(&dev->recover_descriptor));
        seq_printf(m, "  timeout     : %lld\n",
                   atomic64_read(&dev->recover_timeout));
        seq_printf(m, "sync_loss     : %lld\n",
                   atomic64_read(&dev->sync_loss));
        seq_printf(m, "dma_counter   : %lld\n",
                   atomic64_read(&dev->dma_counter));
        seq_printf(m, "dma_ready     : %d\n",
                   READ_ONCE(dev->dma_ready));
        seq_printf(m, "device_dead   : %d\n",
                   smp_load_acquire(&dev->device_dead));
        seq_printf(m, "recover_streak: %d\n",
                   dev->recover_fail_streak);
        {
                unsigned long until = READ_ONCE(dev->recover_backoff_until);
                if (time_after(until, jiffies)) {
                        seq_printf(m, "backoff_remain: %ums\n",
                                   jiffies_to_msecs(until - jiffies));
                }
        }

        for (i = 0; i < MAX_CHANNEL; i++) {
                PT1_CHANNEL *ch = dev->channel[i];
                u64 total_wakeups, wakeups_per_sec;
                unsigned long elapsed_jiffies;
                u32 head, tail, used;

                if (!ch)
                        continue;

                head = smp_load_acquire(&ch->ring.head);
                tail = READ_ONCE(ch->ring.tail);
                used = (head - tail + ring_count) & ring_mask;

                /* wakeup rate 計算 */
                total_wakeups  = atomic64_read(&ch->wakeup_count);
                elapsed_jiffies = jiffies - ch->wakeup_rate_jiffies;
                if (elapsed_jiffies > 0 && elapsed_jiffies < HZ * 3600) {
                        wakeups_per_sec = (total_wakeups - ch->wakeup_rate_snap)
                                          * HZ / elapsed_jiffies;
                } else {
                        wakeups_per_sec = 0;
                }

                seq_printf(m, "ch%d drop=%u overflow=%u ring=%u/%u"
                           " overflow_ring=%d underrun=%d max_fill=%u"
                           " wakeups=%lld (%lld/sec)\n",
                           i, atomic_read(&ch->drop), atomic_read(&ch->overflow), used, ring_count,
                           atomic_read(&ch->overflow_ring),
                           atomic_read(&ch->underrun_count),
                           READ_ONCE(ch->max_fill),
                           total_wakeups, wakeups_per_sec);

                /* ring occupancy histogram */
                seq_printf(m, "  hist 0-25%%=%lld 25-50%%=%lld"
                           " 50-75%%=%lld 75-100%%=%lld\n",
                           atomic64_read(&ch->fill_hist[0]),
                           atomic64_read(&ch->fill_hist[1]),
                           atomic64_read(&ch->fill_hist[2]),
                           atomic64_read(&ch->fill_hist[3]));
        }
        return 0;
}
DEFINE_SHOW_ATTRIBUTE(pt1_stats);

static struct dentry *pt1_debugfs_root;     /* /sys/kernel/debug/pt1/ */

/*
 * pt1_debugfs_init - /sys/kernel/debug/pt1/cardN/ を作成する。
 */
static void pt1_debugfs_init(PT1_DEVICE *dev_conf)
{
        char name[16];
        int i;

        /* 親ディレクトリ /sys/kernel/debug/pt1/ を初回だけ作成 */
        if (!pt1_debugfs_root) {
                pt1_debugfs_root = debugfs_create_dir("pt1", NULL);
                if (IS_ERR_OR_NULL(pt1_debugfs_root)) {
                        pr_warn("pt1: debugfs_create_dir(pt1) failed\n");
                        pt1_debugfs_root = NULL;
                        return;
                }
        }

        snprintf(name, sizeof(name), "card%d", dev_conf->card_number);
        dev_conf->debugfs_dir = debugfs_create_dir(name, pt1_debugfs_root);
        if (IS_ERR_OR_NULL(dev_conf->debugfs_dir)) {
                pr_warn("pt1: debugfs_create_dir(%s) failed\n", name);
                dev_conf->debugfs_dir = NULL;
                return;
        }

        debugfs_create_file("stats", 0444, dev_conf->debugfs_dir,
                            dev_conf, &pt1_stats_fops);

        for (i = 0; i < MAX_CHANNEL; i++) {
                char chname[16];
                snprintf(chname, sizeof(chname), "ring_fill_%d", i);
                debugfs_create_file(chname, 0444, dev_conf->debugfs_dir,
                                    dev_conf->channel[i], &pt1_ring_fill_fops);
        }
}

static void pt1_debugfs_remove(PT1_DEVICE *dev_conf)
{
        debugfs_remove_recursive(dev_conf->debugfs_dir);
        dev_conf->debugfs_dir = NULL;
}
static int      pt1_dma_init(struct pci_dev *pdev, PT1_DEVICE *dev_conf)
{
        int             lp ;
        void    *ptr ;

        for(lp = 0 ; lp < DMA_RING_SIZE ; lp++){
                ptr = dma_alloc_coherent(&pdev->dev, DMA_SIZE, &dev_conf->ring_dma[lp], GFP_KERNEL);
                if(ptr == NULL){
                        printk(KERN_INFO "PT1:DMA ALLOC ERROR\n");
                        /* 確保済み分を解放してから返る */
                        while (--lp >= 0)
                                dma_free_coherent(&pdev->dev, DMA_SIZE,
                                                  dev_conf->dmaptr[lp],
                                                  dev_conf->ring_dma[lp]);
                        return -ENOMEM;
                }
                dev_conf->dmaptr[lp] = ptr ;
        }

        return pt1_makering(pdev, dev_conf);
}
static int      pt1_dma_free(struct pci_dev *pdev, PT1_DEVICE *dev_conf)
{

        int             lp ;
        int             lp2 ;

        for(lp = 0 ; lp < DMA_RING_SIZE ; lp++){
                if(dev_conf->dmaptr[lp] != NULL){
                        dma_free_coherent(&pdev->dev, DMA_SIZE,
                                                                dev_conf->dmaptr[lp], dev_conf->ring_dma[lp]);
                        for(lp2 = 0 ; lp2 < DMA_RING_MAX ; lp2++){
                                if((dev_conf->dmactl[lp])->data[lp2] != NULL){
                                        dma_free_coherent(&pdev->dev, DMA_SIZE,
                                                                                (dev_conf->dmactl[lp])->data[lp2],
                                                                                (dev_conf->dmactl[lp])->ring_dma[lp2]);
                                }
                        }
                }
        }
        return 0 ;
}
static int pt1_pci_init_one (struct pci_dev *pdev,
                                     const struct pci_device_id *ent)
{
        int                     rc ;
        int                     lp ;
        int                     minor ;
        u16                     cmd ;
        PT1_DEVICE      *dev_conf ;
        PT1_CHANNEL     *channel ;
        int i;
        struct resource *dummy;

        rc = pci_enable_device(pdev);
        if (rc)
                return rc;

        /*
         * FIX: pci_set_dma_mask() は Linux 5.18 で削除された。
         * dma_set_mask_and_coherent() を使用する。
         * これにより streaming DMA と coherent DMA の両方に32bitマスクが設定される。
         */
        rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (rc) {
                printk(KERN_ERR "PT1:DMA MASK ERROR");
                return rc;
        }

        pci_read_config_word(pdev, PCI_COMMAND, &cmd);
        if (!(cmd & PCI_COMMAND_MASTER)) {
                printk(KERN_INFO "Attempting to enable Bus Mastering\n");
                pci_set_master(pdev);
                pci_read_config_word(pdev, PCI_COMMAND, &cmd);
                if (!(cmd & PCI_COMMAND_MASTER)) {
                        printk(KERN_ERR "Bus Mastering is not enabled\n");
                        return -EIO;
                }
        }
        printk(KERN_INFO "Bus Mastering Enabled.\n");

        dev_conf = kzalloc(sizeof(PT1_DEVICE), GFP_KERNEL);
        if(!dev_conf){
                printk(KERN_ERR "PT1:out of memory !");
                return -ENOMEM ;
        }
        for (i = 0; i < DMA_RING_SIZE; i++) {
                dev_conf->dmactl[i] = kzalloc(sizeof(DMA_CONTROL), GFP_KERNEL);
                if(!dev_conf->dmactl[i]){
                        int j;
                        for (j = 0; j < i; j++) {
                                kfree(dev_conf->dmactl[j]);
                        }
                        kfree(dev_conf);
                        printk(KERN_ERR "PT1:out of memory !");
                        return -ENOMEM ;
                }
        }

        switch(ent->device) {
        case PCI_PT1_ID:
                dev_conf->cardtype = PT1;
                break;
        case PCI_PT2_ID:
                dev_conf->cardtype = PT2;
                break;
        default:
                break;
        }

        // PCIアドレスをマップする
        dev_conf->mmio_start = pci_resource_start(pdev, 0);
        dev_conf->mmio_len = pci_resource_len(pdev, 0);
        dummy = request_mem_region(dev_conf->mmio_start, dev_conf->mmio_len, DEV_NAME);
        if (!dummy) {
                printk(KERN_ERR "PT1:cannot request iomem  (0x%llx).\n", (unsigned long long) dev_conf->mmio_start);
                goto out_err_regbase;
        }

        dev_conf->regs = ioremap(dev_conf->mmio_start, dev_conf->mmio_len);
        if (!dev_conf->regs){
                printk(KERN_ERR "pt1:Can't remap register area.\n");
                release_mem_region(dev_conf->mmio_start, dev_conf->mmio_len);
                goto out_err_regbase;
        }
        // 初期化処理
        if(xc3s_init(dev_conf->regs, dev_conf->cardtype)){
                printk(KERN_ERR "Error xc3s_init\n");
                goto out_err_fpga;
        }
        // チューナリセット
        settuner_reset(dev_conf->regs, dev_conf->cardtype, LNB_OFF, TUNER_POWER_ON_RESET_ENABLE);
        msleep(100);

        settuner_reset(dev_conf->regs, dev_conf->cardtype, LNB_OFF, TUNER_POWER_ON_RESET_DISABLE);
        msleep(100);
        mutex_init(&dev_conf->lock);

        // Tuner 初期化処理
        for(lp = 0 ; lp < MAX_TUNER ; lp++){
                rc = tuner_init(dev_conf->regs, dev_conf->cardtype, &dev_conf->lock, lp);
                if(rc < 0){
                        printk(KERN_ERR "Error tuner_init\n");
                        goto out_err_fpga;
                }
        }
        // 初期化完了
        for(lp = 0 ; lp < MAX_CHANNEL ; lp++){
                set_sleepmode(dev_conf->regs, &dev_conf->lock,
                                                i2c_address[lp], channeltype[lp], TYPE_SLEEP);

                msleep(100);
        }
        rc = alloc_chrdev_region(&dev_conf->dev, 0, MAX_CHANNEL, DEV_NAME);
        if(rc < 0){
                goto out_err_fpga;
        }

        // 初期化
        init_waitqueue_head(&dev_conf->dma_wait_q);

        minor = MINOR(dev_conf->dev) ;
        dev_conf->base_minor = minor ;
        for(lp = 0 ; lp < MAX_PCI_DEVICE ; lp++){
                printk(KERN_INFO "PT1:device[%d]=%p\n", lp, device[lp]);
                if(device[lp] == NULL){
                        device[lp] = dev_conf ;
                        dev_conf->card_number = lp;
                        break ;
                }
        }
        for(lp = 0 ; lp < MAX_CHANNEL ; lp++){
                cdev_init(&dev_conf->cdev[lp], &pt1_fops);
                dev_conf->cdev[lp].owner = THIS_MODULE;
                /*
                 * FIX: cdev_add() の戻り値を確認していなかった。
                 * 失敗時はこのチャンネルの登録を諦め、既に確保済みの
                 * リソース(channel[0..lp-1] とその ring buffer)を
                 * out_err_v4l で解放してから抜ける。
                 */
                rc = cdev_add(&dev_conf->cdev[lp],
                         MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + lp)), 1);
                if (rc) {
                        printk(KERN_ERR "PT1:cdev_add failed (ch=%d, rc=%d)\n", lp, rc);
                        goto out_err_v4l;
                }
                channel = kzalloc(sizeof(PT1_CHANNEL), GFP_KERNEL);
                if(!channel){
                        printk(KERN_ERR "PT1:out of memory !");
                        /*
                         * FIX: 直接 return していたため、既に確保済みの
                         * dev_conf / dmactl[] / channel[0..lp-1] /
                         * ring buffer / regs / mem_region / chrdev_region
                         * が全てリークしていた。
                         */
                        cdev_del(&dev_conf->cdev[lp]);
                        goto out_err_v4l;
                }

                // 共通情報
                mutex_init(&channel->lock);
                // 待ち状態を解除
                WRITE_ONCE(channel->req_dma, FALSE);
                // マイナー番号設定
                channel->minor = MINOR(dev_conf->dev) + lp ;
                // 対象のI2Cデバイス
                channel->address = i2c_address[lp] ;
                channel->type = channeltype[lp] ;
                // 実際のチューナ番号
                channel->channel = real_channel[lp] ;
                channel->ptr = dev_conf ;
                WRITE_ONCE(channel->size, 0);
                dev_conf->channel[lp] = channel ;

                init_waitqueue_head(&channel->wait_q);

                /*
                 * circular ring buffer を vzalloc で確保する。
                 * channel->buf は legacy フィールドとして NULL のまま残す
                 * (vfree ガードが out_err_v4l にあるため)。
                 */
                channel->ring.buffer = vzalloc((size_t)ring_count * PACKET_SIZE);
                if (!channel->ring.buffer) {
                        /*
                         * FIX: この時点で cdev_add() は成功済みなので
                         * cdev_del() で戻してから抜ける。
                         */
                        cdev_del(&dev_conf->cdev[lp]);
                        goto out_err_v4l;
                }
                WRITE_ONCE(channel->ring.head, 0);
                WRITE_ONCE(channel->ring.tail, 0);
                spin_lock_init(&channel->ring.overflow_lock);

                /* stats 初期化 */
                {
                        int j;
                        for (j = 0; j < 4; j++)
                                atomic64_set(&channel->fill_hist[j], 0);
                        atomic64_set(&channel->wakeup_count, 0);
                        channel->wakeup_rate_jiffies = jiffies;
                        channel->wakeup_rate_snap    = 0;
                        WRITE_ONCE(channel->last_wake_jiffies, jiffies);
                        atomic_set(&channel->overflow_ring, 0);
                        atomic_set(&channel->underrun_count, 0);
                        atomic_set(&channel->overflow, 0);
                        WRITE_ONCE(channel->max_fill, 0);
                }

                switch(channel->type){
                        case CHANNEL_TYPE_ISDB_T:
                                channel->maxsize = CHANNEL_DMA_SIZE ;
                                WRITE_ONCE(channel->pointer, 0);
                                break ;
                        case CHANNEL_TYPE_ISDB_S:
                                channel->maxsize = BS_CHANNEL_DMA_SIZE ;
                                WRITE_ONCE(channel->pointer, 0);
                                break ;
                }
                printk(KERN_INFO "PT1:card_number = %d\n",
                       dev_conf->card_number);
                /*
                 * FIX: device_create() のシグネチャは Linux 2.6.27 以降で統一されており
                 * バージョン分岐は不要。古い #if ブロックを除去してクリーンに。
                 */
                device_create(pt1video_class,
                              NULL,
                              MKDEV(MAJOR(dev_conf->dev),
                                    (MINOR(dev_conf->dev) + lp)),
                              NULL,
                              "pt1video%u",
                              MINOR(dev_conf->dev) + lp +
                              dev_conf->card_number * MAX_CHANNEL);
        }

        if(pt1_dma_init(pdev, dev_conf) < 0){
                goto out_err_dma;
        }

        /* DMA watchdog 初期化 */
        atomic64_set(&dev_conf->dma_counter, 0);
        atomic64_set(&dev_conf->recover_count, 0);
        atomic64_set(&dev_conf->sync_loss, 0);
        atomic64_set(&dev_conf->recover_stall, 0);
        atomic64_set(&dev_conf->recover_sync_loss, 0);
        atomic64_set(&dev_conf->recover_descriptor, 0);
        atomic64_set(&dev_conf->recover_timeout, 0);
        atomic_set(&dev_conf->recovering, 0);
        dev_conf->recover_fail_streak   = 0;
        WRITE_ONCE(dev_conf->recover_backoff_until, jiffies);
        WRITE_ONCE(dev_conf->device_dead, false);
        WRITE_ONCE(dev_conf->last_dma_jiffies, jiffies);
        smp_store_release(&dev_conf->dma_ready, true);

        /*
         * dma_cpu が指定されている場合は kthread_create → kthread_bind → wake_up_process の
         * 順で CPU を確定してから起動する。
         * kthread_run は create + wakeup を同時に行うため bind 前に1回走る可能性がある。
         */
        if (dma_cpu >= 0 && cpu_online(dma_cpu)) {
                /* DMA thread */
                dev_conf->kthread = kthread_create(pt1_thread, dev_conf, "pt1");
                if (IS_ERR(dev_conf->kthread)) {
                        rc = PTR_ERR(dev_conf->kthread);
                        dev_conf->kthread = NULL;
                        goto out_err_dma;
                }
                kthread_bind(dev_conf->kthread, dma_cpu);
                wake_up_process(dev_conf->kthread);

                /* watchdog thread */
                dev_conf->watchdog_task = kthread_create(pt1_watchdog, dev_conf, "pt1_wd");
                if (IS_ERR(dev_conf->watchdog_task)) {
                        pr_warn("pt1: failed to create watchdog thread\n");
                        dev_conf->watchdog_task = NULL;
                } else {
                        kthread_bind(dev_conf->watchdog_task, dma_cpu);
                        wake_up_process(dev_conf->watchdog_task);
                }

                pr_info("pt1: card%d DMA/watchdog threads pinned to CPU%d\n",
                        dev_conf->card_number, dma_cpu);
        } else {
                if (dma_cpu >= 0)
                        pr_warn("pt1: dma_cpu=%d not online, ignoring\n", dma_cpu);

                /* CPU 固定なし: kthread_run で起動 */
                dev_conf->kthread = kthread_run(pt1_thread, dev_conf, "pt1");
                if (IS_ERR(dev_conf->kthread)) {
                        rc = PTR_ERR(dev_conf->kthread);
                        dev_conf->kthread = NULL;
                        goto out_err_dma;
                }

                dev_conf->watchdog_task = kthread_run(pt1_watchdog, dev_conf, "pt1_wd");
                if (IS_ERR(dev_conf->watchdog_task)) {
                        pr_warn("pt1: failed to start watchdog thread\n");
                        dev_conf->watchdog_task = NULL;
                }
        }

        pt1_debugfs_init(dev_conf);

        pci_set_drvdata(pdev, dev_conf);
        return 0;

out_err_dma:
        pt1_dma_free(pdev, dev_conf);
out_err_v4l:
        /*
         * FIX: ここまでに cdev_add() / device_create() が成功済みだった
         * チャンネル(0..lp-1。全チャンネル完了後に out_err_dma 経由で
         * 来た場合は lp == MAX_CHANNEL)を cdev_del() / device_destroy()
         * で戻す。従来はここが抜けており、プローブ失敗時に登録済みの
         * cdev やデバイスノードが残存したままになっていた。
         */
        for (i = 0; i < lp; i++) {
                device_destroy(pt1video_class,
                               MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + i)));
                cdev_del(&dev_conf->cdev[i]);
        }
        for(lp = 0 ; lp < MAX_CHANNEL ; lp++){
                if(dev_conf->channel[lp] != NULL){
                        if(dev_conf->channel[lp]->ring.buffer != NULL){
                                vfree(dev_conf->channel[lp]->ring.buffer);
                        }
                        kfree(dev_conf->channel[lp]);
                }
        }
        /*
         * FIX: out_err_v4l に来る時点では alloc_chrdev_region() は
         * 必ず成功済み(このラベルへ来る失敗は cdev_add/kzalloc/vzalloc/
         * dma_init/kthread のいずれかで、全て alloc_chrdev_region の
         * 後段)。従来はここで確保済みのメジャー/マイナー番号領域を
         * 解放しておらず、probe 失敗のたびにリークしていた。
         */
        unregister_chrdev_region(dev_conf->dev, MAX_CHANNEL);
out_err_fpga:
        writel(0xb0b0000, dev_conf->regs);
        writel(0, dev_conf->regs + CFG_REGS_ADDR);
        iounmap(dev_conf->regs);
        release_mem_region(dev_conf->mmio_start, dev_conf->mmio_len);
        for (i = 0; i < DMA_RING_SIZE; i++) {
                kfree(dev_conf->dmactl[i]);
        }
        kfree(dev_conf);
out_err_regbase:
        /*
         * FIX: request_mem_region() / ioremap() 失敗時にここへ来るが、
         * この時点で dev_conf と dmactl[] は既に確保済みのため
         * 解放しないとリークしていた。
         */
        for (i = 0; i < DMA_RING_SIZE; i++)
                kfree(dev_conf->dmactl[i]);
        kfree(dev_conf);
        return -EIO;

}

static void pt1_pci_remove_one(struct pci_dev *pdev)
{

        int             lp ;
        __u32   val ;
        PT1_DEVICE      *dev_conf = (PT1_DEVICE *)pci_get_drvdata(pdev);
        int             i;

        if(dev_conf){
                pt1_debugfs_remove(dev_conf);

                /*
                 * kthread_stop() の前に全 waitqueue を起こす。
                 * wait_event_timeout で寝ている read() / pt1_thread が
                 * 最大 500ms 待たずに即座に停止できる。
                 */
                {
                        int wi;
                        wake_up_all(&dev_conf->dma_wait_q);
                        for (wi = 0; wi < MAX_CHANNEL; wi++) {
                                if (dev_conf->channel[wi])
                                        wake_up_all(&dev_conf->channel[wi]->wait_q);
                        }
                }

                if(dev_conf->watchdog_task) {
                        kthread_stop(dev_conf->watchdog_task);
                        dev_conf->watchdog_task = NULL;
                }
                if(dev_conf->kthread) {
                        kthread_stop(dev_conf->kthread);
                        /* synchronize_rcu() は RCU read-side を使っていないため不要 */
                        dev_conf->kthread = NULL;
                }

                // DMA終了
                writel(0x08080000, dev_conf->regs);
                for(lp = 0 ; lp < 10 ; lp++){
                        val = readl(dev_conf->regs);
                        if(!(val & (1 << 6))){
                                break ;
                        }
                        msleep(100);
                }
                pt1_dma_free(pdev, dev_conf);
                for(lp = 0 ; lp < MAX_CHANNEL ; lp++){
                        if(dev_conf->channel[lp] != NULL){
                                cdev_del(&dev_conf->cdev[lp]);
                                if(dev_conf->channel[lp]->ring.buffer != NULL)
                                        vfree(dev_conf->channel[lp]->ring.buffer);
                                kfree(dev_conf->channel[lp]);
                        }
                        device_destroy(pt1video_class,
                                       MKDEV(MAJOR(dev_conf->dev),
                                             (MINOR(dev_conf->dev) + lp)));
                }

                unregister_chrdev_region(dev_conf->dev, MAX_CHANNEL);
                writel(0xb0b0000, dev_conf->regs);
                writel(0, dev_conf->regs + CFG_REGS_ADDR);
                settuner_reset(dev_conf->regs, dev_conf->cardtype, LNB_OFF, TUNER_POWER_OFF);
                release_mem_region(dev_conf->mmio_start, dev_conf->mmio_len);
                iounmap(dev_conf->regs);
                for (i = 0; i < DMA_RING_SIZE; i++) {
                        kfree(dev_conf->dmactl[i]);
                }
                device[dev_conf->card_number] = NULL;
                kfree(dev_conf);
        }
        pci_set_drvdata(pdev, NULL);
}
#ifdef CONFIG_PM

static int pt1_pci_suspend (struct pci_dev *pdev, pm_message_t state)
{
        PT1_DEVICE *dev_conf = pci_get_drvdata(pdev);

        if (!dev_conf)
                return 0;

        /*
         * watchdog が DMA 未初期化状態で recover を走らせないよう
         * 先に dma_ready を落とす。
         */
        WRITE_ONCE(dev_conf->dma_ready, false);

        /* DMA を安全に停止 */
        pt1_dma_stop(dev_conf);
        pt1_wait_dma_idle(dev_conf);

        pci_save_state(pdev);
        pci_disable_device(pdev);

        return 0;
}

static int pt1_pci_resume (struct pci_dev *pdev)
{
        PT1_DEVICE *dev_conf = pci_get_drvdata(pdev);
        int rc;

        if (!dev_conf)
                return 0;

        /*
         * device_dead: 永続的な DMA 障害で dead 状態のカードは
         * resume しても回復しないため、DMA 再起動を行わない。
         * rmmod → modprobe で完全に再初期化する必要がある。
         */
        if (smp_load_acquire(&dev_conf->device_dead)) {
                pr_warn("pt1: card%d: resume skipped, device is dead\n",
                        dev_conf->card_number);
                return 0;
        }

        rc = pci_enable_device(pdev);
        if (rc) {
                pr_err("pt1: pci_enable_device failed on resume (%d)\n", rc);
                return rc;
        }
        pci_restore_state(pdev);
        pci_set_master(pdev);

        /*
         * interrupt clear: suspend 前に積まれた stale IRQ status を明示的に捨てる。
         * PT2 は resume 直後に古い IRQ status が残る場合があり、
         * watchdog が「DMA alive」と誤認する原因になる。
         * readl → writel(clear) → readl(flush) の3ステップで安全にクリア。
         */
        {
                __u32 status = readl(dev_conf->regs);
                /* status のクリア: bit4(counter reset) を立てて古い状態をリセット */
                writel(status | 0x00000010, dev_conf->regs);
                readl(dev_conf->regs);  /* PCI write flush */
                mb();
        }

        /*
         * descriptor を全域ゼロクリアしてから DMA を再起動する。
         * reset_dma() が memset + counter reset + DMA start を行う。
         */
        reset_dma(dev_conf);

        /* watchdog 監視を再開 */
        WRITE_ONCE(dev_conf->last_dma_jiffies, jiffies);
        atomic64_set(&dev_conf->dma_counter, 0);
        /*
         * smp_store_release: reset_dma() の descriptor 書き込み完了が
         * watchdog から可視化されることを保証する。
         * watchdog 側の smp_load_acquire と対になる。
         */
        smp_store_release(&dev_conf->dma_ready, true);

        return 0;
}

#endif /* CONFIG_PM */


static struct pci_driver pt1_driver = {
        .name           = DRV_NAME,
        .probe          = pt1_pci_init_one,
        .remove         = pt1_pci_remove_one,
        .id_table       = pt1_pci_tbl,
#ifdef CONFIG_PM
        .suspend        = pt1_pci_suspend,
        .resume         = pt1_pci_resume,
#endif /* CONFIG_PM */

};


static int __init pt1_pci_init(void)
{
        printk(KERN_INFO "%s", version);

        /* ring_count の検証: power of 2、かつ 256〜1048576 の範囲 */
        if (ring_count < 256 || ring_count > 1048576 ||
            (ring_count & (ring_count - 1)) != 0) {
                pr_warn("pt1: invalid ring_count=%u, using default %u\n",
                        ring_count, TS_RING_COUNT);
                ring_count = TS_RING_COUNT;
        }
        ring_mask = ring_count - 1;
        pr_info("pt1: ring_count=%u (~%uKB/ch)\n",
                ring_count, ring_count * PACKET_SIZE / 1024);

        /*
         * class_create() は Linux 6.4 で THIS_MODULE 引数が削除された。
         * 対象カーネルが 6.8(Ubuntu 24.04) 以降のみになったため
         * LINUX_VERSION_CODE 分岐は不要（常に真）。削除して単純化。
         */
        pt1video_class = class_create(DRIVERNAME);
        if (IS_ERR(pt1video_class))
                return PTR_ERR(pt1video_class);
        return pci_register_driver(&pt1_driver);
}


static void __exit pt1_pci_cleanup(void)
{
        pci_unregister_driver(&pt1_driver);
        class_destroy(pt1video_class);
        debugfs_remove_recursive(pt1_debugfs_root);
        pt1_debugfs_root = NULL;
}

module_init(pt1_pci_init);
module_exit(pt1_pci_cleanup);