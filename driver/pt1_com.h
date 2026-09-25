#ifndef		__PT1_COM_H__
#define		__PT1_COM_H__
/***************************************************************************/
/* I2Cデータ位置定義                                                       */
/***************************************************************************/
#define		MAX_CHANNEL			4		// チャネル数
#define		FALSE		0
#define		TRUE		1
#define		MAX_TUNER			2		//チューナ数
enum{
	CHANNEL_TYPE_ISDB_S,
	CHANNEL_TYPE_ISDB_T,
	CHANNEL_TYPE_MAX
};
/***************************************************************************/
/* 状態                                                                    */
/***************************************************************************/
enum{
	STATE_STOP,			// 初期化直後
	STATE_START,		// 通常
	STATE_FULL			// ストッパー
};
/***************************************************************************/
/* ログ出力(debug モジュールパラメータで詳細度を制御)                      */
/*   0:静音 1:通常(既定) 2:詳細。エラー/警告は debug に関係なく出力する      */
/*   各 .c で pr_fmt を定義しているため、接頭辞は "pt1_drv: " になる          */
/***************************************************************************/
extern int pt1_debug;
/* pt1_debug >= lvl のときだけ出力する。_rl は ratelimit 付き */
#define pt1_log(lvl, fmt, ...) \
	do { \
		if (READ_ONCE(pt1_debug) >= (lvl)) \
			pr_info(fmt, ##__VA_ARGS__); \
	} while (0)
#define pt1_log_rl(lvl, fmt, ...) \
	do { \
		if (READ_ONCE(pt1_debug) >= (lvl)) \
			pr_info_ratelimited(fmt, ##__VA_ARGS__); \
	} while (0)
#endif
