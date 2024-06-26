#include "utils/indicators.hpp"

namespace indicators {

IndeterminateProgressBar getIndeterminateProgressBar(int width) {
  return IndeterminateProgressBar{option::BarWidth{width},
                                  indicators::option::Fill{"····"},
                                  indicators::option::Lead{"<==>"},
                                  PROGRESS_STYLE};
}


ProgressBar getProgressBar(int total, int width) {
  return ProgressBar{option::BarWidth{width},
                     option::MaxProgress{total},
                     PROGRESS_STYLE,
                     PROGRESS_SHOWTIME};
}


ProgressSpinner getProgressSpinner(int total) {
  return ProgressSpinner{option::MaxProgress{total},
                         PROGRESS_STYLE};
}

}
