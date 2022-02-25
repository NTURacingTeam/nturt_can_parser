#ifndef CAN_PARSER__H
#define CAN_PARSER__H

// define result state here
#ifndef OK_ERR
#define OK_ERR

#define OK -1
#define ERR 0

#endif
// define some parameters
#define Frame_NUM 8 // how many kinds of frame here? plus 1 then.
#define DATA_NUM 11 // how many kinds of data here? plus 1 then.
// define Frame type here
#define FB1 1  // front box
#define FB2 2  // front box
#define RBX 3  // rear box
#define INV 4  // inverter
#define BMS 5  // BMS
#define HIA 6  // accelerometer frame from Honeywell IMU
#define HIG 7  // gyroscope frame from Honeywell IMU
#define HIC 8  // pose frame from Honeywell IMU
// and so on...
// define data type here
#define ACC 1  // accelerometer
#define GYR 2  // gyrscope
#define CMP 3  // compass
#define FWS 4  // front wheel speed
#define RWS 5  // rear wheel speed
#define THR 6  // throttle 
#define BRK 7  // brake
#define STR 8  // steer
#define BTP 9  // battery temp
#define BVT 10 // battery voltage
// and so on...

class NTURT_CAN_parser {
public:
    /* parameter:
     *   Frame type
     *   CAN id
     * return:
     *   return OK or ERR
     */
    int assign_id(int type, int id){
        int idsize = sizeof(id_)/sizeof(int); // Should be Frame_NUM
        if(type<idsize){
            id_[type] = id;
            return OK;
        }
        else{
            return ERR;
        }
    }

    /* parameter:
     *   CAN id
     * return:
     *   Frame type
     */
    int get_type(int id){
        for(int i=1;i<=Frame_NUM;i++){
            if(id_[i] == id){
                return id_[i];
            }
        }
        return ERR;
    }
    
    /* parameter:
     *   Frame type
     * return:
     *   CAN id
     */
    int get_id(int type){
        return id_[type];
    }

    /* parameters:
     *   Frame type
     *   CAN data (uint[8])
     * return:
     *   OK or ERR
     * It decode data according to Frame type,
     * and it store the result to the class member variable.
     */
    int decode(int type, int* data);
    
    /* parameters:
     *   Frame type
     *   data to-be-encoded (double [NUM])
     *   CAN data (uint[8])
     * return:
     *   CAN id
     * It encode tbe (data to-be-encoded) according to Frame type,
     * and it store the result to data (int[8]).
     */
    int encode(int type, double* tbe, int* data);
    
    /* parameters:
     *   variables to-be-updated(i.e. sensor data)
     *   Here they are accelerometer data
     * return:
     *   OK or ERR
     */
    int get_ACC(double &ax, double &ay, double &az);
    
    /* parameters:
     *   variables to-be-updated(i.e. sensor data)
     *   Here they are gyroscope data
     * return:
     *   OK or ERR
     */
    int get_GYR(double &gx, double &gy, double &gz);

    /* parameters:
     *   variables to-be-updated(i.e. sensor data)
     *   Here they are pose (compass) data (roll pitch yaw)
     * return:
     *   OK or ERR
     */
    int get_CMP(double &cx, double &cy, double &cz);

    /* parameters:
     *   variables to-be-updated(i.e. sensor data)
     *   Here they are front wheel speed data (left/right)
     * return:
     *   OK or ERR
     */
    int get_FWS(double &fws_l, double &fws_r);
    
    /* parameters:
     *   variables to-be-updated(i.e. sensor data)
     *   Here they are rear wheel speed data (left/right)
     * return:
     *   OK or ERR
     */
    int get_RWS(double &rws_l, double &rws_r);
    
    /* parameters:
     *   variables to-be-updated(i.e. sensor data)
     *   Here they are throttle data
     * return:
     *   OK or ERR
     */
    int get_THR(double &th_1, double &th_2);
    
    /* parameters:
     *   variables to-be-updated(i.e. sensor data)
     *   Here they are brake data
     * return:
     *   OK or ERR
     */
    int get_BRK(double &brk);
    
    /* parameters:
     *   variables to-be-updated(i.e. sensor data)
     *   Here they are steer data
     * return:
     *   OK or ERR
     */
    int get_STR(double &str);
    // and BMS data... 
private:
    /* The id of different CAN message frame.
     * ex: id[FBX] is the id of front box messages
     */
    int id_[Frame_NUM];
    
    /* The flag of different sensor data, which indicates if there is new data.
     * ex: flag[ACC] means if we got new accelerometer data.
     * If we got message from a node,
     * then flag[data from that node] will be 1.
     * If we accessed this data via get_XXX(),
     * then flag[the data] will be 0
     */
    int flag_[DATA_NUM];
    
    // declare every data as member function
    // IMU
    double accx_, accy_, accz_, gyrx_, gyry_, gyrz_, cmpx_, cmpy_, cmpz_;
    // Front box
    double brk_, thr1_, thr2_, str_;
    // Wheel Speed
    double fws_l_, fws_r_, rws_l_, rws_r_;
    // Suspension ride height
    double frh_l_, frh_r_, rrh_l_, rrh_r_;
    // Rear box
    // BMS
    // Inverter
    // Dashboard
    // and so on.......
};

typedef NTURT_CAN_parser Parser;

#endif