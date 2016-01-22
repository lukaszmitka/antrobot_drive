#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "rs232.h"

int send_command(std::string command, std::vector<double> *position);
int init_comport(int port_number);
int close_comport(int port_number);