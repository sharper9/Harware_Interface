#ifndef BIT_UTILS_H
#define BIT_UTILS_H

class Utils_Base
{
public:
	int zero_one(int input_val) {if (input_val == 0) return 0; else return 1;}
};

class Set_Reset:public Utils_Base
{
private: 
	int val;
public: 
	Set_Reset() {val = 0;}
	int set(int input_val) {if (zero_one(input_val)==1) val = 1; return val;}
	int reset(int input_val) {if (zero_one(input_val)==1) val = 0; return val;}
	int get_val() {return val;}
};

class Leading_Edge_Latch:public Utils_Base
{
private:
	int last_val;
	int LE_val;
	int first_pass;
public:
	Leading_Edge_Latch() {last_val = 0; LE_val = 0; first_pass = 1;}
	int LE_Latch(int input_val) {if(first_pass==1){last_val = zero_one(input_val);} LE_val = (zero_one(input_val)==1/*!=last_val*/) && (last_val==0); last_val = zero_one(input_val); first_pass = 0; return LE_val;}
	int get_val() {return LE_val;}
};

class Trailing_Edge_Latch:public Utils_Base
{
private:
	int last_val;
	int TE_val;
public:
	Trailing_Edge_Latch() {last_val = 0; TE_val = 0;}
	int TE_Latch(int input_val) {TE_val = (zero_one(input_val)!=last_val) && (last_val==1); last_val = input_val; return TE_val;}
	int get_val() {return TE_val;}
};

class Toggle:public Utils_Base
{
private:
    int val;
    Leading_Edge_Latch latch;
public:
    Toggle() {val = 0;}
    //int toggle(int input_val) {if(input_val && val==0){val=1;} else if(input_val && val==1){val=0;} return val;}
    int toggle(int input_val) {latch.LE_Latch(input_val); if(latch.get_val() && val==0){val=1;} else if(latch.get_val() && val==1){val=0;} return val;}
    int get_val() {return val;}
};

#endif /* BIT_UTILS_H */
