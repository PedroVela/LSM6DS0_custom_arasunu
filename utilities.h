int16_t rawData(uint8_t buffer_[2]){
    uint16_t aux = ((uint16_t )buffer_[1] << 8) | (uint16_t )(buffer_[0]);
    int16_t y = *(int16_t*)&aux;
    return y;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}