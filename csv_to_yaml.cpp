//#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <algorithm>
#include <fstream>

#define SPACE2 "\n  "
#define SPACE4 "\n    "
#define SPACE6 "\n      "
#define SPACE8 "\n        "

using namespace std;

string append = "";
ifstream in("./rule.csv");
ofstream out("./output.yaml");

string buff;
string word;
vector<vector<string>> buff_vec(72, vector<string> (12));

void s2() {
    append += SPACE2;
}
void s4() {
    append += SPACE4;
}
void s6() {
    append += SPACE6;
}
void s8() {
    append += SPACE8;
}
void print_data(int i) {
    append = "";
    s6();
    append += "data:";
    s8();
    append += "name: ";
    append += buff_vec[i][1];
    s8();
    append += "start_byte: ";
    append += buff_vec[i][5];
    s8();
    append += "end_byte: ";
    append += buff_vec[i][6];
    s8();
    append += "start_bit: ";
    append += buff_vec[i][7];
    s8();
    append += "stop_bit: ";
    append += buff_vec[i][8];
    s8();
    append += "default: 0";
    s8();
    append += "resolution: ";
    append += buff_vec[i][10];
    s8();
    append += "offset: ";
    append += buff_vec[i][11];
    s8();
    append += "is_signed: ";
    append += buff_vec[i][9];
    s8();
    append += "is_little_endian: ";
    string little_endian = "true";
    if (buff_vec[i][4] == "3") {
        little_endian = "false";
    }
    append += little_endian;
    out << append;
}

int main() {


    for(int i=0; (getline(in, buff)); i++) {
        stringstream ss(buff);
        //cout << buff << "\n";
        for (int j=0; (getline(ss, word, ',')); j++) {
            //cout << word << "\n";
            buff_vec[i][j] = word;
        }
    }
    out << "can:";
    //out << SPACE2 << "test\n";

    for (int i=0; i<72; i++) {
        if (i == 0 || buff_vec[i][2] != buff_vec[i-1][2]) {
            s2();
            append += "frame:";
            s4();
            append += "name: ";
            append += buff_vec[i][0];
            s4();
            append += "id: ";
            append += buff_vec[i][2];
            s4();
            string extended = "false";
            if (buff_vec[i][2].length() == 10) {
                extended = "true"; 
            }
            append += "is_extended_id: " + extended;
            s4();
            append += "dlc: 8";
            s4();
            append += "frequency: 0";
            s4();
            append += "dataset:";
            out << append;
        }
        print_data(i);
    }
    
    return 0;
}
