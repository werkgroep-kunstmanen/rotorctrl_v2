void calc_sat_earth_v2(struct tm *cur_tm,int cur_ms, // time
                    KEPLER *kepler,               // sat. parameters
                    EPOINT *pos_earth,            // pos. earth (rotation), may be NULL
                    EPOINT *pos_sat,              // pos. satellite, may be NULL
                    EPOINT *pos_subsat);           // sub-satellite position w.r.t. earth
void elevazim2xy(DIR *satdir,ROTOR *rot);
double calceleazim_v2(struct tm cur_tm,int ms,EPOINT *pos_subsat,EPOINT *pos_sat,EPOINT *refpos,DIR *satdir);
void load_default_refpos(EPOINT *refpos);
void load_default_kepler(KEPLER *kepler);
boolean calc_pos(GOTO_VAL *gotoval,KEPLER *kepler,EPOINT *refpos);
int calc_sgp4_const(KEPLER *kepler,boolean);
