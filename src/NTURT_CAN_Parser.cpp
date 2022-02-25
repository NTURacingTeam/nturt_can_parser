#include <NTURT_CAN_Parser.hpp>

int Parser::decode(int type, int* data){
    if(type == FB1){
        thr1_  = data[0]*100/255;
        thr2_  = data[1]*100/255;
        brk_   = data[2]*100/255;
        str_   = data[3]*100/255;
        fws_l_ = (data[4]*256+data[5])*0.002;
        fws_r_ = (data[6]*256+data[7])*0.002;
        // flags
        flag_[FWS] = 1;
        flag_[THR] = 1;
        flag_[BRK] = 1;
        flag_[STR] = 1;
        return OK_PARSER;
    }
    else if(type == FB2){
        // TODO
        // No such message yet
        return OK_PARSER;
    }
    else if(type == RBX){
        rws_l_ = (data[2]*256+data[3])*0.002;
        rws_r_ = (data[4]*256+data[5])*0.002;
        // flags
        flag_[RWS] = 1;
        return OK_PARSER;
    }
    else if(type == HIA){
        accx_ = ((data[3]*256+data[2])-32000)/100;
        accy_ = ((data[1]*256+data[0])-32000)/100;
        accz_ = ((data[5]*256+data[4])-32000)/100;
        // flags
        flag_[ACC] = 1;
        return OK_PARSER;
    }
    else if(type == HIG){
        gyrx_ = ((data[3]*256+data[2])-32000)/128;
        gyry_ = ((data[1]*256+data[0])-32000)/128;
        gyrz_ = ((data[5]*256+data[4])-32000)/128;
        // flags
        flag_[GYR] = 1;
        return OK_PARSER;
    }
    else if(type == HIC){
        cmpx_ = ((data[2]*65536+data[1]*256+data[0])-8192000)/32768;
        cmpy_ = ((data[5]*65536+data[4]*256+data[3])-8192000)/32768;
        // no cmpz_ for Honeywell IMU
        // flags
        flag_[CMP] = 1;
        return OK_PARSER;
    }
    // still have to decode BMS and INVerter
    else{
        // Wrong Message
        return ERR_PARSER;
    }
}

int Parser::encode(int type, double* tbe, int* data){
    if(type==INV){
        // encode torque data here
        return OK_PARSER;
    }
    else{
        return ERR_PARSER;
    }
}

int Parser::get_ACC(double &ax, double &ay, double &az){
    if(flag_[ACC]){
        ax = accx_;
        ay = accy_;
        az = accz_;
        flag_[ACC] = 0;
        return OK_PARSER;
    }
    else{
        // not decoded yet
        return ERR_PARSER;
    }
}

int Parser::get_GYR(double &gx, double &gy, double &gz){
    if(flag_[GYR]){
        gx = gyrx_;
        gy = gyry_;
        gz = gyrz_;
        flag_[GYR] = 0;
        return OK_PARSER;
    }
    else{
        // not decoded yet
        return ERR_PARSER;
    }
}

int Parser::get_CMP(double &cx, double &cy, double &cz){
    if(flag_[CMP]){
        cx = cmpx_;
        cy = cmpy_;
        cz = cmpz_;
        flag_[CMP] = 0;
        return OK_PARSER;
    }
    else{
        // not decoded yet
        return ERR_PARSER;
    }
}

int Parser::get_FWS(double &fws_l, double &fws_r){
    if(flag_[FWS]){
        fws_l = fws_l_;
        fws_r = fws_r_;
        flag_[FWS] = 0;
        return OK_PARSER;
    }
    else{
        // not decoded yet
        return ERR_PARSER;
    }
}

int Parser::get_RWS(double &rws_l, double &rws_r){
    if(flag_[RWS]){
        rws_l = rws_l_;
        rws_r = rws_r_;
        flag_[RWS] = 0;
        return OK_PARSER;
    }
    else{
        // not decoded yet
        return ERR_PARSER;
    }
}

int Parser::get_THR(double &th_1, double &th_2){
    if(flag_[THR]){
        th_1 = thr1_;
        th_2 = thr2_;
        flag_[THR] = 0;
        return OK_PARSER;
    }
    else{
        // not decoded yet
        return ERR_PARSER;
    }
}
    
int Parser::get_BRK(double &brk){
    if(flag_[BRK]){
        brk = brk_;
        flag_[BRK] = 0;
        return OK_PARSER;
    }
    else{
        // not decoded yet
        return ERR_PARSER;
    }
}

int Parser::get_STR(double &str){
    if(flag_[STR]){
        str = str_;
        flag_[STR] = 0;
        return OK_PARSER;
    }
    else{
        // not decoded yet
        return ERR_PARSER;
    }

}
