/**
 * @file s_curve_params.c
 * @author 19816
 * @brief S 曲线运动参数计算源文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#include <math.h>

#define S_CURVE_PARAMS_MAIN
#include "s_curve_params.h"

#ifndef S_CURVE_FMAX
#define S_CURVE_FMAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef S_CURVE_FMIN
#define S_CURVE_FMIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/* #define ERR_MIN        0.000001f */
#define ERR_MIN		0.0001f
#define L_CALC_CYC	32
#define T_CALC_CYC	16

/* -------- 单段（单调速度变化段）的计算参数 v_in -> v_out -------- */
typedef struct {
	float sign;
    unsigned char  reached_amax; /* 是否触达amax：1=是，0=否 */
    float  t_j;          /* 正负加加速度对称段的持续时间 */
    float  t_a;          /* 加/减速的恒速时间（可能为0） */
    float  dur;          /* 总时间 = 2*t_j + t_a */
    float  dx;           /* 位移 */
} mono_segment_params_t;



/**
  * @brief  将输入值限制为不小于指定下限。
  * @param  x 输入值。
  * @param  mn 最小允许值。
  * @retval float 限幅后的结果。
  */
static inline float ClampMin(float x, float mn)
{
	return (x < mn) ? mn : x;
}

/**
  * @brief  解析计算单调速度变化段的时间和位移参数。
  * @param  handle 单调段计算结果输出指针。
  * @param  v_in 初始速度。
  * @param  v_out 目标速度。
  * @param  amax 最大加速度。
  * @param  j 最大加加速度。
  * @retval 无
  * @note   根据速度差自动判断是否包含恒加速度段。
  */
static void MonoSegmentAnalytic(mono_segment_params_t* handle, float v_in, float v_out,
                                         float amax, float j)
{
    /* mono_segment_params_t r = {0}; */

	float sign;
	float dv,dv_tri;

	float tj,ta,v1,v2;
	float dx1,dx2,dx3;

	/* dv = fabs(v_out - v_in); */
	dv=v_out - v_in;
	if (dv >= 0)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
		dv=-dv;
	}

	handle->sign=sign;

	dv_tri = (amax*amax)/j;

    if (dv < dv_tri)
    {
        /* 纯jerk控制，无恒加速度 */
        handle->reached_amax = 0;        /* 未触达标志 */
        handle->t_j = sqrt(dv / j);      /* 加减速时间 */
        handle->t_a = 0.0f;              /* 恒速时间 */
        handle->dur = 2.0f * handle->t_j;/* 总时间 */

        /* 位移：利用对称合并后的表达式 */
		handle->dx  = 2.0f * v_in * handle->t_j + (j * handle->t_j*handle->t_j*handle->t_j)*sign;
    }
	else
	{
        /* jerk+恒加速度，触达amax */
        handle->reached_amax = 1;
        handle->t_j = amax / j;
        handle->t_a = (dv - amax*amax/j) / amax;
        if (handle->t_a < 0)
        {
            handle->t_a = 0.0f;
        }  /* 数值安全 */
        handle->dur = 2.0f * handle->t_j + handle->t_a;

        /* 分段位移 */
        tj = handle->t_j, ta = handle->t_a;
        v1 = v_in + 0.5f * amax*amax / j * sign;    /* 正加加速度段结束时速度 */
        v2 = v1   + amax * ta *sign;              /* 恒加速结束时的速度 */

        dx1 = v_in * tj + (1.0/6.0) * j * tj*tj*tj *sign;
        dx2 = v1   * ta + 0.5f * amax * ta*ta *sign;
        dx3 = v2   * tj + (0.5f * amax * tj*tj - (1.0f/6.0f) * j * tj*tj*tj)*sign;

        handle->dx = dx1 + dx2 + dx3;
    }
}

/**
  * @brief  计算无匀速段情况下从 v0 到峰值速度再减速到 0 的总位移。
  * @param  v0 初始速度。
  * @param  vp 峰值速度。
  * @param  amax 最大加速度。
  * @param  j 最大加加速度。
  * @retval float 加速段和减速段位移之和。
  */
static float SCurveDistanceNoCruise(float v0, float vp,
                                 float amax, float j)
{
    mono_segment_params_t acc;
	mono_segment_params_t dec;
	MonoSegmentAnalytic(&acc, v0, vp, amax, j);
    MonoSegmentAnalytic(&dec, vp, 0.0f, amax, j);
    return acc.dx + dec.dx;
}

/**
  * @brief  根据距离和运动限制求解完整 S 曲线参数。
  * @param  handle S 曲线参数输出指针。
  * @param  input S 曲线输入约束参数指针。
  * @retval 无
  * @note   会根据距离是否足够自动选择有匀速段或无匀速段的解。
  */
void SolveScurveParams(func_s_params_t* handle, func_s_inputs_t* input)
{
    float d_accdec_vmax,dx_acc,dx_dec,rem;
	mono_segment_params_t acc;
	mono_segment_params_t dec;
	int it;

	/* handle handle = {0}; */
    /* 最小值限制 */
    input->v0   = ClampMin(input->v0,   0.0);
    input->vmax = ClampMin(input->vmax, 0.0);
    input->amax = ClampMin(input->amax, 1e-12);
    input->j    = ClampMin(input->j,    1e-12);

    /* 先计算：若峰值速度取vmax，所需时间位移 */
    d_accdec_vmax = SCurveDistanceNoCruise(input->v0, input->vmax, input->amax, input->j);
	/* acc = MonoSegmentAnalytic(input->v0, input->vmax, input->amax, input->j); */
	/* dec = MonoSegmentAnalytic(input->vmax, 0.0, input->amax, input->j); */
	/* d_accdec_vmax = acc.dx + dec.dx; */

	handle->a_sign=1;
	handle->d_sign=-1;

    if (input->l > d_accdec_vmax)
	{
        /* --- 情况A：距离足够大，加速到vmax再减速 => 需要匀速段 --- */
        handle->vc = input->vmax;

        /* 计算加速段和减速段的时间和位移 */
        MonoSegmentAnalytic(&acc,input->v0, handle->vc, input->amax, input->j);
        MonoSegmentAnalytic(&dec,handle->vc, 0.0, input->amax, input->j);

        /* 匀速时间：由位移闭合，t_c = (l - dx_acc - dx_dec)/vp */
        dx_acc = acc.dx;
        dx_dec = dec.dx;
        rem = input->l - (dx_acc + dx_dec);
        handle->t_c = rem / S_CURVE_FMAX(handle->vc, 1e-12f);
        if (handle->t_c < 0)
        {
            handle->t_c = 0.0f;
        }  /* 数值安全 */

        /* 输出参数 */
        handle->acc_reach_amax = acc.reached_amax;
        handle->dec_reach_amax = dec.reached_amax;

        handle->t_ja = acc.t_j; handle->t_aa = acc.t_a;
        handle->t_jd = dec.t_j; handle->t_ad = dec.t_a;
        handle->dx_acc   = dx_acc;
        handle->dx_dec   = dx_dec;
        handle->dx_total = dx_acc + handle->vc*handle->t_c + dx_dec;
        handle->tim = acc.dur + handle->t_c + dec.dur;
    }
	else
	{
        /* --- 情况B：距离不足 => 无匀速段，计算vp<=vmax使恰好走完l --- */
        float lo = S_CURVE_FMAX(input->v0, 1e-12f);
        float hi = S_CURVE_FMAX(lo,    input->vmax);
        /* 二分法：vp单调递增函数（距离单调增函数） */
        for (it=0; it<80; ++it)
		{
            float mid = 0.5f*(lo+hi);
            float dmid = SCurveDistanceNoCruise(input->v0, mid, input->amax, input->j);
            if (dmid < input->l)
			{
				lo = mid;
			}
			else
			{
				hi = mid;
			}
            if (fabs(hi-lo) <= 1e-12f * S_CURVE_FMAX(1.0f, mid))
            {
                break;
            }
        }
        handle->vc = 0.5f*(lo+hi);
        handle->t_c = 0.0f;

        MonoSegmentAnalytic(&acc,input->v0, handle->vc, input->amax, input->j);
        MonoSegmentAnalytic(&dec,handle->vc, 0.0,    input->amax, input->j);

        handle->acc_reach_amax = acc.reached_amax;
        handle->dec_reach_amax = dec.reached_amax;
        handle->t_ja = acc.t_j; handle->t_aa = acc.t_a;
        handle->t_jd = dec.t_j; handle->t_ad = dec.t_a;

        handle->dx_acc   = acc.dx;
        handle->dx_dec   = dec.dx;
        handle->dx_total = acc.dx + dec.dx;   /* 无匀速段 */
        handle->tim = acc.dur + dec.dur;
    }
}

#define S_TLV_SUCCESS  1  /* 成功 */
#define S_TLV_T_SHORT  (-1)  /* 时间太短 */
#define S_TLV_V02V1    2  /* 初速度转末速度的时间太短 */
#define S_TLV_L_VC     3  /* 匀速模式，有匀速段 */
#define S_TLV_L_NVC    4  /* 匀速模式，无匀速段 */
#define S_TLV_L_SHORT  (-2)  /* 匀速模式，距离太短 */
/**
  * @brief  根据初末速度、距离和时间约束计算 S 曲线参数。
  * @param  handle S 曲线参数输出指针。
  * @param  input TLV 输入约束参数指针。
  * @retval S_TLV_SUCCESS 参数计算成功。
  * @retval S_TLV_T_SHORT 给定时间过短。
  * @retval S_TLV_V02V1 仅执行初速度到末速度的速度迁移。
  * @retval S_TLV_L_VC 存在匀速段。
  * @retval S_TLV_L_NVC 无匀速段。
  * @retval S_TLV_L_SHORT 距离过短，无法满足约束。
  */
int FuncS_Tlv(func_s_params_t* handle, func_s_tlv_in_t* input)
{
	float l,vc,dl,hi,lo,f,tc;
	int r;

	unsigned int it;

	mono_segment_params_t acc;
	mono_segment_params_t dec={0};
	r = 0;        /* 返回值 */
	vc = input->vc;  /* 巡航速度 */
	tc = 0;      /* 巡航时间 */


	if (input->v0 > input->vmax)
	{
		input->v0=input->vmax;
	}
	if (input->v1 > input->vmax)
	{
		input->v1=input->vmax;
	}
	if (input->vc > input->vmax)
	{
		input->vc=input->vmax;
	}

	while(r==0)
	{
		if(input->t==0)
		{
/* =================================================================================================== */
/* -------------------------- 从v0速度开始，以vc为巡航速度，末速度v1，走完l --------------------------- */
			vc=input->vc;
			MonoSegmentAnalytic(&acc, input->v0, vc, input->amax, input->j); /* 计算速度跃迁的时间和长度 */
			MonoSegmentAnalytic(&dec, vc, input->v1, input->amax, input->j); /* 计算速度跃迁的时间和长度 */

			if((acc.dx + dec.dx) <= input->l)
			{
				f = input->l - acc.dx -dec.dx;
				f /=  vc;
				tc = f;
				r=S_TLV_L_VC;
				break;
			}
			else
			{
				/* 二分法继续逼近巡航速度 */
				hi=vc;
				lo=0;
				l=input->l;
				for(it=0; it<L_CALC_CYC; ++it)
				{
					vc=(hi+lo)/2;

					MonoSegmentAnalytic(&acc,input->v0,vc,input->amax,input->j);
					MonoSegmentAnalytic(&dec,vc,input->v1,input->amax,input->j);

					dl = (acc.dx + dec.dx) - l;

					if(fabs(dl)<ERR_MIN)
					{
						break;
					}

					if (dl > 0)  /* 加减速所需总距离大于L时 */
					{
						hi = vc;      /* 此时巡航速度太高，降低巡航速度 */
					}
					else
					{
						lo=vc;
					}
				}
				if(it == L_CALC_CYC)
				{
					r=S_TLV_L_SHORT;
				}
				else
				{
					r=S_TLV_L_NVC;
				}
				break;
			}
		}
/* ------------------------ 从v0速度开始，以vc为巡航速度，走完l，末速度v1 ------------------------- */
/* =================================================================================================== */
		MonoSegmentAnalytic(&acc, input->v0, input->v1, input->amax, input->j); /* 计算速度跃迁的时间和长度 */
		if((acc.dur > input->t) && (input->t!=0))
		{
			r = S_TLV_T_SHORT;  /* 从初速度迁移到末速度，所需时间太短，无法完成速度变化 */
			break;
		}
		if(input->l == 0)
		{
			r=S_TLV_V02V1;
		}
/* =================================================================================================== */
/* ----------------------------------- 已知v0,v1,t,l，求vc最优解 ------------------------------------ */

/* ----------------------------------- 使用二分法寻找vc最优解 ------------------------------------ */
		hi=input->vmax;
		lo=0;
		l=input->l;
		tc=0;
		for(it=0; it<L_CALC_CYC; ++it)
		{

			vc=(hi+lo)/2;

			MonoSegmentAnalytic(&acc,input->v0,vc,input->amax,input->j);
			MonoSegmentAnalytic(&dec,vc,input->v1,input->amax,input->j);

			if (((acc.dx + dec.dx) > l) || ((acc.dur + dec.dur) > input->t)) /* 加减速所需总距离或总时间超过限制 */
			{
				hi = vc;      /* 此时巡航速度太高，降低巡航速度 */
			}
			else
			{
				tc=input->t - (acc.dur + dec.dur);
				dl=(acc.dx + dec.dx + tc*vc) - l;

				if (fabs(dl) <= ERR_MIN)
				{
					handle->vc=vc;
					break;
				}
				else if(dl < 0)
				{
					lo=vc;
				}
				else
				{
					hi=vc;
				}
			}
		}

		if(it!=L_CALC_CYC)
		{
			r = S_TLV_SUCCESS;  /* 成功找到 */
		}
		else
		{
			/* 在规定的时间距离内无法满足，放开 t 限制重新计算 vc */
			hi=input->vmax;
			lo=0;
			l=input->l;
			for(it=0; it<T_CALC_CYC; ++it)
			{
				vc=(hi+lo)/2;
				MonoSegmentAnalytic(&acc,input->v0,vc,input->amax,input->j);
				MonoSegmentAnalytic(&dec,vc,input->v1,input->amax,input->j);

				if (((acc.dx + dec.dx) > l))  /* 加减速所需总距离超过L时 */
				{
					hi = vc;      /* 距离太大，降低巡航速度 */
				}
				else
				{
					dl=l-(acc.dx + dec.dx);
					tc=dl/vc;

					if((input->t - (acc.dur + dec.dur + tc)) < 0)
					{
						lo = vc;      /* 时间太长，提高巡航速度 */
					}
					else
					{
						hi=vc;
					}
				}
			}
			r=S_TLV_T_SHORT;
			break;
		}
	}
	handle->v0=input->v0;
	handle->v1=input->v1;
	handle->a_sign=acc.sign;
	handle->t_ja=acc.t_j;
	handle->t_aa=acc.t_a;
	handle->t_c=tc;
	handle->vc=vc;
	handle->d_sign=dec.sign;
	handle->t_jd=dec.t_j;
	handle->t_ad=dec.t_a;
	handle->dx_acc=acc.dx;
	handle->dx_dec=dec.dx;
	handle->dx_total=acc.dx+dec.dx+tc*vc;
	handle->tim=handle->t_ja*2 + handle->t_aa + handle->t_c + handle->t_jd*2 + handle->t_ad;
	return(r);
}

/**
  * @brief  计算速度从 v0 变化到 v1 所需时间和距离。
  * @param  v0 初始速度。
  * @param  v1 目标速度。
  * @param  amax 最大加速度。
  * @param  j 最大加加速度。
  * @retval func_s_calc_tl_t 速度变化所需时间和位移。
  */
func_s_calc_tl_t FuncS_CalcTl(float v0,float v1,float amax,float j)
{
	mono_segment_params_t acc;
	func_s_calc_tl_t tl;

	MonoSegmentAnalytic(&acc, v0, v1, amax, j);
	tl.l = acc.dx;
	tl.t = acc.dur;
	return(tl);
}

