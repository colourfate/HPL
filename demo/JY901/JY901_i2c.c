#include "api_os.h"
#include "api_debug.h"
#include "api_event.h"
#include "api_hal_i2c.h"

#include "JY901_i2c.h"

void JY901_read_acc(struct SAcc *std_acc)
{
	uint8_t buf[6];
	I2C_ReadMem(I2C_JY901, JY901_SLAVE_ADDR, AX, 6, buf, 6, I2C_DEFAULT_TIME_OUT);
	std_acc->a[0] = ((short)buf[1]<<8) | buf[0];
	std_acc->a[1] = ((short)buf[3]<<8) | buf[2];
	std_acc->a[2] = ((short)buf[5]<<8) | buf[4];
}

void JY901_read_gyro(struct SGyro *std_gyro)
{
	uint8_t buf[6];
	I2C_ReadMem(I2C_JY901, JY901_SLAVE_ADDR, GX, 6, buf, 6, I2C_DEFAULT_TIME_OUT);
	std_gyro->w[0] = ((short)buf[1]<<8) | buf[0];
	std_gyro->w[1] = ((short)buf[3]<<8) | buf[2];
	std_gyro->w[2] = ((short)buf[5]<<8) | buf[4];
}

void JY901_read_mag(struct SMag *std_mag)
{
	uint8_t buf[6];
	I2C_ReadMem(I2C_JY901, JY901_SLAVE_ADDR, HX, 6, buf, 6, I2C_DEFAULT_TIME_OUT);
	std_mag->h[0] = ((short)buf[1]<<8) | buf[0];
	std_mag->h[1] = ((short)buf[3]<<8) | buf[2];
	std_mag->h[2] = ((short)buf[5]<<8) | buf[4];
}

void JY901_read_angle(struct SAngle *std_angle)
{
	uint8_t buf[6];
	I2C_ReadMem(I2C_JY901, JY901_SLAVE_ADDR, Roll, 6, buf, 6, I2C_DEFAULT_TIME_OUT);
	std_angle->Angle[0] = ((short)buf[1]<<8) | buf[0];
	std_angle->Angle[1] = ((short)buf[3]<<8) | buf[2];
	std_angle->Angle[2] = ((short)buf[5]<<8) | buf[4];
}

void JY901_read_q(struct SQ *std_q)
{
	uint8_t buf[8];
	I2C_ReadMem(I2C_JY901, JY901_SLAVE_ADDR, Q0, 8, buf, 8, I2C_DEFAULT_TIME_OUT);
	std_q->q[0] = ((short)buf[1]<<8) | buf[0];
	std_q->q[1] = ((short)buf[3]<<8) | buf[2];
	std_q->q[2] = ((short)buf[5]<<8) | buf[4];
	std_q->q[3] = ((short)buf[7]<<8) | buf[6];
}
