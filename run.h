void straight(float len, float acc, float max_sp, float end_sp);
void back_straight(float len, float acc, float max_sp, float end_sp);
void turn(int deg, float ang_acc, float max_om, short dir);
void check_straight(float len, float acc, float max_sp, float end_sp);
void slalom(int deg, float ang_accel, float max_ang_velocity, short dir);
void slalom_2(int deg, float ang_accel, float max_ang_velocity, short dir);
void slalom_straight(float len, float acc, float max_sp, float end_sp);
void slalom_straight_2(float len, float acc, float max_sp, float end_sp);
void Kanayama_sla(short dir);
void run_test(float Vl, float Vr, int time);
void run_test_2(int t1, int t2, int t3, int t4, int t5, int t6);
void check_FF_run(int t1, int t2, int t3);

