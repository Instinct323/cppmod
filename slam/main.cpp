#include "utils.h"
#include <thread>

int main() {
  Timer timer;

  this_thread::sleep_for(chrono::duration<double>(1));
  cout << timer << endl;
}
