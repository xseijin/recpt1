#ifndef		__PT1_IOCTL_H__
#define		__PT1_IOCTL_H__
/***************************************************************************/
/* チャンネル周波数情報構造体定義                                          */
/***************************************************************************/
typedef	struct	_frequency{
	int		frequencyno ;			// 周波数テーブル番号
	int		slot ;					// スロット番号／加算する周波数
}FREQUENCY;

/*
 * FIX: ISDB-S(BS/CS)で、EPGデータ等から得た実際のTSID値を直接指定して
 * ロックするための構造体。SET_CHANNEL の slot(0〜7のインデックス)とは
 * 異なり、こちらは16bitのTSID値そのものを渡す。
 * ts_lock() はハードウェアに直接そのTSID値を指示して結果をポーリング
 * 確認する作りなので、bs_tune() が返す ts_id[] 配列から事前に
 * slot番号を調べる必要がない。ISDB-T(地上波)では使用しない
 * (1周波数=1TSのため、frequencyno のみで一意に決まる)。
 */
typedef	struct	_frequency_tsid{
	int		frequencyno ;			// 周波数テーブル番号(SET_CHANNELと同じ)
	__u16	tsid ;					// 直接指定するTSID値
}FREQUENCY_TSID;

/***************************************************************************/
/* IOCTL定義                                                               */
/***************************************************************************/
#define		SET_CHANNEL	_IOW(0x8D, 0x01, FREQUENCY)
#define		START_REC	_IO(0x8D, 0x02)
#define		STOP_REC	_IO(0x8D, 0x03)
#define		GET_SIGNAL_STRENGTH	_IOR(0x8D, 0x04, int *)
#define		LNB_ENABLE	_IOW(0x8D, 0x05, int)
#define		LNB_DISABLE	_IO(0x8D, 0x06)
/* FIX: TSID直接指定でのチャンネル選局(ISDB-S専用)。既存ioctlとの互換性を保つため新規追加。 */
#define		SET_CHANNEL_TSID	_IOW(0x8D, 0x07, FREQUENCY_TSID)
#endif
