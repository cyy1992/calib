
#ifndef __DMUTILS_H__
#define __DMUTILS_H__

#include "libdmtx/dmtx.h"

#define VARIANCE_YPART_THRESHOLD	1
#define LEAST_SQUARE_LEAST_CNT		10

#define LINE_POINT_NUM				150	//�����ϵ��������ĸ���

typedef struct _dmLine
{
	float k;
	float b;
	uint16_t ls_flag;
}dmLine;

typedef struct _dmLinePoints
{
	DmtxPixelLoc line_point[LINE_POINT_NUM];
	uint16_t line_start;
	uint16_t line_end;
	uint16_t ls_flag;	//0��ʾx��y��������1��ʾ����
	uint16_t line_flag;	//0��ʾbottom,1��ʾleft
}dmLinePoints;

#ifdef __cplusplus
extern "C" {
#endif

/** ֱ�����
 *
 * ������С���˷�����ֱ����ϣ���������ϵ��
 * @param[in]	p 		��Ե�㼯
 * @param[in] 	count 	�㼯�Ĵ�С
 * @param[out] 	b		���ֱ�ߵĽؾ�
 * @param[out]	k		���ֱ�ߵ�б��
 * @return				�жϵ㼯���ֱ�ߵ����ϵ��
 */
float LeastSquare(DmtxPixelLocf* p, uint16_t count, float* b, float* k, int ls_flag);

int16_t get_otsu_thresh(const uint8_t* in_img, uint16_t total);

void IMG_histogram_8(const uint8_t* img, int16_t size, int16_t accumulate, int16_t* t_hist, int16_t* hist);

void IMG_histogram_8_2(const uint8_t* img, const int regionWidth, const int regionHeight, const int regionWidthStep, uint16_t* hist);

#ifdef __cplusplus
}
#endif

#endif
