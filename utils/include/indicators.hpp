#ifndef UTILS__INDICATORS_HPP
#define UTILS__INDICATORS_HPP

#include <indicators/cursor_control.hpp>
#include <indicators/indeterminate_progress_bar.hpp>
#include <indicators/progress_bar.hpp>
#include <indicators/progress_spinner.hpp>

#define PROGRESS_WIDTH 40

#define PROGRESS_SHOWTIME \
  option::ShowElapsedTime{true}, \
  option::ShowRemainingTime{true}

#define PROGRESS_STYLE \
  option::ForegroundColor{Color::blue}, \
  option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}

namespace indicators {

IndeterminateProgressBar getIndeterminateProgressBar(int width = PROGRESS_WIDTH);

ProgressBar getProgressBar(int total, int width = PROGRESS_WIDTH);

ProgressSpinner getProgressSpinner(int total);

}

#endif
