/*********************************************************************************************************************
* CYT4BB Opensourec Library 鍗筹紙 CYT4BB 寮€婧愬簱锛夋槸涓€涓�鍩轰簬瀹樻柟 SDK 鎺ュ彛鐨勭��涓夋柟寮€婧愬簱
* Copyright (c) 2022 SEEKFREE 閫愰�炵�戞妧
*
* 鏈�鏂囦欢鏄� CYT4BB 寮€婧愬簱鐨勪竴閮ㄥ垎
*
* CYT4BB 寮€婧愬簱 鏄�鍏嶈垂杞�浠�
* 鎮ㄥ彲浠ユ牴鎹�鑷�鐢辫蒋浠跺熀閲戜細鍙戝竷鐨� GPL锛圙NU General Public License锛屽嵆 GNU閫氱敤鍏�鍏辫�稿彲璇侊級鐨勬潯娆�
* 鍗� GPL 鐨勭��3鐗堬紙鍗� GPL3.0锛夋垨锛堟偍閫夋嫨鐨勶級浠讳綍鍚庢潵鐨勭増鏈�锛岄噸鏂板彂甯冨拰/鎴栦慨鏀瑰畠
*
* 鏈�寮€婧愬簱鐨勫彂甯冩槸甯屾湜瀹冭兘鍙戞尌浣滅敤锛屼絾骞舵湭瀵瑰叾浣滀换浣曠殑淇濊瘉
* 鐢氳嚦娌℃湁闅愬惈鐨勯€傞攢鎬ф垨閫傚悎鐗瑰畾鐢ㄩ€旂殑淇濊瘉
* 鏇村�氱粏鑺傝�峰弬瑙� GPL
*
* 鎮ㄥ簲璇ュ湪鏀跺埌鏈�寮€婧愬簱鐨勫悓鏃舵敹鍒颁竴浠� GPL 鐨勫壇鏈�
* 濡傛灉娌℃湁锛岃�峰弬闃�<https://www.gnu.org/licenses/>
*
* 棰濆�栨敞鏄庯細
* 鏈�寮€婧愬簱浣跨敤 GPL3.0 寮€婧愯�稿彲璇佸崗璁� 浠ヤ笂璁稿彲鐢虫槑涓鸿瘧鏂囩増鏈�
* 璁稿彲鐢虫槑鑻辨枃鐗堝湪 libraries/doc 鏂囦欢澶逛笅鐨� GPL3_permission_statement.txt 鏂囦欢涓�
* 璁稿彲璇佸壇鏈�鍦� libraries 鏂囦欢澶逛笅 鍗宠�ユ枃浠跺す涓嬬殑 LICENSE 鏂囦欢
* 娆㈣繋鍚勪綅浣跨敤骞朵紶鎾�鏈�绋嬪簭 浣嗕慨鏀瑰唴瀹规椂蹇呴』淇濈暀閫愰�炵�戞妧鐨勭増鏉冨０鏄庯紙鍗虫湰澹版槑锛�
*
* 鏂囦欢鍚嶇О          zf_device_dl1b
* 鍏�鍙稿悕绉�          鎴愰兘閫愰�炵�戞妧鏈夐檺鍏�鍙�
* 鐗堟湰淇℃伅          鏌ョ湅 libraries/doc 鏂囦欢澶瑰唴 version 鏂囦欢 鐗堟湰璇存槑
* 寮€鍙戠幆澧�          IAR 9.40.1
* 閫傜敤骞冲彴          CYT4BB
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
*
* 淇�鏀硅�板綍
* 鏃ユ湡              浣滆€�                澶囨敞
* 2024-01-12       pudding           first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 鎺ョ嚎瀹氫箟锛�
*                   ------------------------------------
*                   妯″潡绠¤剼            鍗曠墖鏈虹�¤剼
*                   SCL                 鏌ョ湅 zf_device_dl1b.h 涓� DL1B_SCL_PIN  瀹忓畾涔�
*                   SDA                 鏌ョ湅 zf_device_dl1b.h 涓� DL1B_SDA_PIN  瀹忓畾涔�
*                   XS                  鏌ョ湅 zf_device_dl1b.h 涓� DL1B_XS_PIN  瀹忓畾涔�
*                   VCC                 5V 鐢垫簮
*                   GND                 鐢垫簮鍦�
*                   ------------------------------------
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _ZF_DEVICE_DL1B_H_
#define _ZF_DEVICE_DL1B_H_

#include "zf_common_typedef.h"

// 闇€瑕佹敞鎰忕殑鏄� DL1B 鏈€楂樻敮鎸� 400KHz 鐨� IIC 閫氫俊閫熺巼
// 闇€瑕佹敞鎰忕殑鏄� DL1B 鏈€楂樻敮鎸� 400KHz 鐨� IIC 閫氫俊閫熺巼
// 闇€瑕佹敞鎰忕殑鏄� DL1B 鏈€楂樻敮鎸� 400KHz 鐨� IIC 閫氫俊閫熺巼

#define DL1B_USE_SOFT_IIC           ( 1 )                                       // 榛樿�や娇鐢ㄨ蒋浠� IIC 鏂瑰紡椹卞姩 寤鸿��浣跨敤杞�浠� IIC 鏂瑰紡
#if DL1B_USE_SOFT_IIC                                                           // 杩欎袱娈� 棰滆壊姝ｅ父鐨勬墠鏄�姝ｇ‘鐨� 棰滆壊鐏扮殑灏辨槸娌℃湁鐢ㄧ殑
//====================================================杞�浠� IIC 椹卞姩====================================================
#define DL1B_SOFT_IIC_DELAY         ( 100 )                                      // 杞�浠� IIC 鐨勬椂閽熷欢鏃跺懆鏈� 鏁板€艰秺灏� IIC 閫氫俊閫熺巼瓒婂揩
#define DL1B_SCL_PIN                (P19_0)                                      // 杞�浠� IIC SCL 寮曡剼 杩炴帴 DL1B 鐨� SCL 寮曡剼
#define DL1B_SDA_PIN                (P19_1)                                      // 杞�浠� IIC SDA 寮曡剼 杩炴帴 DL1B 鐨� SDA 寮曡剼
//====================================================杞�浠� IIC 椹卞姩====================================================
#else
#error "鏆備笉鏀�鎸佺‖浠禝IC閫氳��"
#endif

#define DL1B_XS_PIN                 ( P07_2 )

#define DL1B_INT_ENABLE             ( 0 )                                       // 鏄�鍚﹀惎鐢� INT 寮曡剼 鍚�鐢ㄥ垯浼氳嚜鍔ㄦ洿鏂版暟鎹�
#if DL1B_INT_ENABLE
#define DL1B_INT_PIN                ( P07_3 )
#endif

#define DL1B_TIMEOUT_COUNT          ( 1000 )                                    // DL1B 瓒呮椂璁℃暟

//================================================瀹氫箟 DL1B 鍐呴儴鍦板潃================================================

#define DL1B_DEV_ADDR                                           ( 0x52 >> 1 )   // 0b0101001

#define DL1B_I2C_SLAVE__DEVICE_ADDRESS                          ( 0x0001 )
#define DL1B_GPIO__TIO_HV_STATUS                                ( 0x0031 )
#define DL1B_SYSTEM__INTERRUPT_CLEAR                            ( 0x0086 )
#define DL1B_RESULT__RANGE_STATUS                               ( 0x0089 )
#define DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0     ( 0x0096 )
#define DL1B_FIRMWARE__SYSTEM_STATUS                            ( 0x00E5 )

//================================================瀹氫箟 DL1B 鍐呴儴鍦板潃================================================

extern uint8 dl1b_finsh_flag;
extern uint16 dl1b_distance_mm;

void   dl1b_get_distance (void);

void   dl1b_int_handler  (void);
uint8  dl1b_init         (void);

#endif

