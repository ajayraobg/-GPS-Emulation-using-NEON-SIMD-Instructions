#include <arm_neon.h>

typedef float32x4_t v_4f;  
typedef uint32x4_t v_4u;  
typedef int32x4_t v_4i;  


#define DP1 -0.78515625
#define DP2 -2.4187564849853515625e-4
#define DP3 -3.77489497744594108e-8
#define c_sincof_p0 -1.9515295891E-4
#define c_sincof_p1  8.3321608736E-3
#define c_sincof_p2 -1.6666654611E-1
#define c_coscof_p0  2.443315711809948E-005
#define c_coscof_p1 -1.388731625493765E-003
#define c_coscof_p2  4.166664568298827E-002
#define FOPI 1.27323954473516 



void sincos_neon(v_4f x, v_4f *ysin, v_4f *ycos) { 
  v_4f x1, x2, x3, y;
  v_4u x4;  
  v_4u sign_mask_sin, sign_mask_cos;

  sign_mask_sin = vcltq_f32(x, vdupq_n_f32(0));
  x = vabsq_f32(x);
  y = vmulq_f32(x, vdupq_n_f32(FOPI));
  x4 = vcvtq_u32_f32(y);
  x4 = vaddq_u32(x4, vdupq_n_u32(1));
  x4 = vandq_u32(x4, vdupq_n_u32(~1));
  y = vcvtq_f32_u32(x4);
  v_4u poly_mask = vtstq_u32(x4, vdupq_n_u32(2));
  x1 = vmulq_n_f32(y, DP1);
  x2 = vmulq_n_f32(y, DP2);
  x3 = vmulq_n_f32(y, DP3);
  x = vaddq_f32(x, x1);
  x = vaddq_f32(x, x2);
  x = vaddq_f32(x, x3);
  sign_mask_sin = veorq_u32(sign_mask_sin, vtstq_u32(x4, vdupq_n_u32(4)));
  sign_mask_cos = vtstq_u32(vsubq_u32(x4, vdupq_n_u32(2)), vdupq_n_u32(4));
  v_4f z = vmulq_f32(x,x);
  v_4f y1, y2;
  y1 = vmulq_n_f32(z, c_coscof_p0);
  y2 = vmulq_n_f32(z, c_sincof_p0);
  y1 = vaddq_f32(y1, vdupq_n_f32(c_coscof_p1));
  y2 = vaddq_f32(y2, vdupq_n_f32(c_sincof_p1));
  y1 = vmulq_f32(y1, z);
  y2 = vmulq_f32(y2, z);
  y1 = vaddq_f32(y1, vdupq_n_f32(c_coscof_p2));
  y2 = vaddq_f32(y2, vdupq_n_f32(c_sincof_p2));
  y1 = vmulq_f32(y1, z);
  y2 = vmulq_f32(y2, z);
  y1 = vmulq_f32(y1, z);
  y2 = vmulq_f32(y2, x);
  y1 = vsubq_f32(y1, vmulq_f32(z, vdupq_n_f32(0.5f)));
  y2 = vaddq_f32(y2, x);
  y1 = vaddq_f32(y1, vdupq_n_f32(1));  
  v_4f ys = vbslq_f32(poly_mask, y1, y2);
  v_4f yc = vbslq_f32(poly_mask, y2, y1);
  *ysin = vbslq_f32(sign_mask_sin, vnegq_f32(ys), ys);
  *ycos = vbslq_f32(sign_mask_cos, yc, vnegq_f32(yc));
}

v_4f sin_neon(v_4f x) {
  v_4f ysin, ycos; 
  sincos_neon(x, &ysin, &ycos); 
  return ysin;
}

v_4f cos_neon(v_4f x) {
  v_4f ysin, ycos; 
  sincos_neon(x, &ysin, &ycos); 
  return ycos;
}