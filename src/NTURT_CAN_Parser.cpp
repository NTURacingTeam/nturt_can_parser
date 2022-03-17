#include <NTURT_CAN_Parser.hpp>

int Parser::init_parser(){
    assign_id(_CP_FB1, 0x080AD091);
    assign_id(_CP_FB2, 0x080AD092);
    assign_id(_CP_RB1, 0x080AD093);
    assign_id(_CP_RB2, 0x080AD094);
    // assign_id(_CP_OIA, 0x08f02de2);
    // assign_id(_CP_OIG, 0x0cf02ae2);
    // assign_id(_CP_OIC, 0x0cf029e2);
    // assign_id(_CP_OIS, 0x0cf029e2);
    assign_id(_CP_HIA, 0x08f02de2);
    assign_id(_CP_HIG, 0x0cf02ae2);
    assign_id(_CP_HIC, 0x0cf029e2);
    std::cout<<"Parser Initialize~~"<<std::endl;
    print_id();
    return OK;
}

int Parser::decode(int type, int* data){
    if(type == _CP_FB1){
        fws_l_ = (data[0]*256+data[1])*0.002;
        fws_r_ = (data[2]*256+data[3])*0.002;
        // flags
        flag_[_CP_FWS] = 1;
        return OK;
    }
    else if(type == _CP_FB2){
        thr1_  = data[0]*100/255;
        thr2_  = data[1]*100/255;
        brk_   = data[2]*100/255;
        str_   = data[3]*100/255;
        // flags
        flag_[_CP_THR] = 1;
        flag_[_CP_BRK] = 1;
        flag_[_CP_STR] = 1;
        // No such message yet
        return OK;
    }
    else if(type == _CP_RB1){
        rws_l_ = (data[0]*256+data[1])*0.002;
        rws_r_ = (data[2]*256+data[3])*0.002;
        // flags
        flag_[_CP_RWS] = 1;
        return OK;
    }
    else if(type == _CP_RB2){
        rws_l_ = (data[2]*256+data[3])*0.002;
        rws_r_ = (data[4]*256+data[5])*0.002;
        // flags
        flag_[_CP_RWS] = 1;
        return OK;
    }
    else if(type == _CP_HIA){
        accx_ = ((data[3]*256+data[2])-32000)/100;
        accy_ = ((data[1]*256+data[0])-32000)/100;
        accz_ = ((data[5]*256+data[4])-32000)/100;
        // flags
        flag_[_CP_ACC] = 1;
        return OK;
    }
    else if(type == _CP_HIG){
        gyrx_ = ((data[3]*256+data[2])-32000)/128;
        gyry_ = ((data[1]*256+data[0])-32000)/128;
        gyrz_ = ((data[5]*256+data[4])-32000)/128;
        // flags
        flag_[_CP_GYR] = 1;
        return OK;
    }
    else if(type == _CP_HIC){
        cmpx_ = ((data[2]*65536+data[1]*256+data[0])-8192000)/32768;
        cmpy_ = ((data[5]*65536+data[4]*256+data[3])-8192000)/32768;
        // no cmpz_ for Honeywell IMU
        // flags
        flag_[_CP_CMP] = 1;
        return OK;
    }
    // still have to decode BMS and INVerter
    else{
        // Wrong Message
        return ERR;
    }
}

int Parser::encode(int type, double* tbe, int* data){
    if(type==_CP_INV){
        // encode torque data here
        return OK;
    }
    else if(type == _CP_FB1){
        int fws_l = tbe[0]/0.002;
        int fws_r = tbe[1]/0.002;
        int ft_l1 = (tbe[2]-(-273.15))/0.02;
        int ft_l2 = (tbe[3]-(-273.15))/5.12;
        int ft_r1 = (tbe[4]-(-273.15))/0.02;
        int ft_r2 = (tbe[5]-(-273.15))/5.12;
        data[0] = fws_l/256;
        data[1] = fws_l%256;
        data[2] = fws_r/256;
        data[3] = fws_r%256;
        data[4] = ft_l1%256;
        data[5] = ft_l2%256;
        data[6] = ft_r1%256;
        data[7] = ft_r2%256;
        // flags
        flag_[_CP_FWS] = 1;
        flag_[_CP_FWT] = 1;
        return OK;
    }
    else if(type == _CP_FB2){
        int fws_l = tbe[0]/0.002;
        int fws_r = tbe[1]/0.002;
        int ft_l1 = (tbe[2]-(-273.15))/0.02;
        int ft_l2 = (tbe[3]-(-273.15))/5.12;
        int ft_r1 = (tbe[4]-(-273.15))/0.02;
        int ft_r2 = (tbe[5]-(-273.15))/5.12;
        data[0] = fws_l/256;
        data[1] = fws_l%256;
        data[2] = fws_r/256;
        data[3] = fws_r%256;
        data[4] = ft_l1%256;
        data[5] = ft_l2%256;
        data[6] = ft_r1%256;
        data[7] = ft_r2%256;
        // flags
        flag_[_CP_FWS] = 1;
        flag_[_CP_FWT] = 1;
        return OK;
    }
    else if(type == _CP_RB1){
        int rws_l = tbe[0]/0.002;
        int rws_r = tbe[1]/0.002;
        int rt_l1 = (tbe[2]-(-273.15))/0.02;
        int rt_l2 = (tbe[3]-(-273.15))/5.12;
        int rt_r1 = (tbe[4]-(-273.15))/0.02;
        int rt_r2 = (tbe[5]-(-273.15))/5.12;
        data[0] = rws_l/256;
        data[1] = rws_l%256;
        data[2] = rws_r/256;
        data[3] = rws_r%256;
        data[4] = rt_l1%256;
        data[5] = rt_l2%256;
        data[6] = rt_r1%256;
        data[7] = rt_r2%256;
        // flags
        flag_[_CP_RWS] = 1;
        flag_[_CP_RWT] = 1;
        return OK;
    }
    else{
        return ERR;
    }
}

#ifdef BOOST_ARRAY
int Parser::encode(int type, double* tbe, boost::array<unsigned char, 8> &data){
    if(type==_CP_INV){
        // encode torque data here
        return OK;
    }
    else{
        return ERR;
    }
}
#endif

int Parser::get_ACC(double &ax, double &ay, double &az){
    if(flag_[_CP_ACC]){
        ax = accx_;
        ay = accy_;
        az = accz_;
        flag_[_CP_ACC] = 0;
        return OK;
    }
    else{
        // not decoded yet
        return ERR;
    }
}

int Parser::get_GYR(double &gx, double &gy, double &gz){
    if(flag_[_CP_GYR]){
        gx = gyrx_;
        gy = gyry_;
        gz = gyrz_;
        flag_[_CP_GYR] = 0;
        return OK;
    }
    else{
        // not decoded yet
        return ERR;
    }
}

int Parser::get_CMP(double &cx, double &cy, double &cz){
    if(flag_[_CP_CMP]){
        cx = cmpx_;
        cy = cmpy_;
        cz = cmpz_;
        flag_[_CP_CMP] = 0;
        return OK;
    }
    else{
        // not decoded yet
        return ERR;
    }
}

int Parser::get_FWS(double &fws_l, double &fws_r){
    if(flag_[_CP_FWS]){
        fws_l = fws_l_;
        fws_r = fws_r_;
        flag_[_CP_FWS] = 0;
        return OK;
    }
    else{
        // not decoded yet
        return ERR;
    }
}

int Parser::get_RWS(double &rws_l, double &rws_r){
    if(flag_[_CP_RWS]){
        rws_l = rws_l_;
        rws_r = rws_r_;
        flag_[_CP_RWS] = 0;
        return OK;
    }
    else{
        // not decoded yet
        return ERR;
    }
}

int Parser::get_THR(double &th_1, double &th_2){
    if(flag_[_CP_THR]){
        th_1 = thr1_;
        th_2 = thr2_;
        flag_[_CP_THR] = 0;
        return OK;
    }
    else{
        // not decoded yet
        return ERR;
    }
}
    
int Parser::get_BRK(double &brk){
    if(flag_[_CP_BRK]){
        brk = brk_;
        flag_[_CP_BRK] = 0;
        return OK;
    }
    else{
        // not decoded yet
        return ERR;
    }
}

int Parser::get_STR(double &str){
    if(flag_[_CP_STR]){
        str = str_;
        flag_[_CP_STR] = 0;
        return OK;
    }
    else{
        // not decoded yet
        return ERR;
    }

}
