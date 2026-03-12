#include "slave.h"

int main() {

  SLAVE_init();
    
  while(1) {
    SLAVE_state_machine();
  }
}

