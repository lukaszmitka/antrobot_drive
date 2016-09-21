#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "rs232.h"

int send_command(std::string command);
int init_comport(int port_number);
int close_comport(int port_number);
void check_for_response(std::vector<double> *position);