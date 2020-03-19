typedef struct {
	float p;
	float i;
	float d;
  float tpa_const;
	float i_range;
	float error; //private
	float last_error; //private
	uint64_t current_micros; //private
	uint64_t last_micros; //private
	uint64_t elapsed_seconds; //private
}PID_TypeDef;