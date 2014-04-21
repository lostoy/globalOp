#include <numeric>

#include "Constant.h"

/* The length of point in bytes in .bin files produced from Depth2Ply */
const int Constant::POINT_XYZRGBA_LENGTH_IN_BYTES = 17;

/* The length of point in bytes in .bin files used by Xuetong Sun in preview */
const int Constant::POINT_XYZRGBA_LENGTH_IN_BYTES_XT = 40;

/**/
const size_t Constant::DEFAULT_BIN_FILE_SIZE = ((size_t)1) << 23;

/* Bad point for invalid depth point */
const float Constant::BAD_POINT_VALUE = std::numeric_limits<float>::quiet_NaN();

Constant::Constant()
{
}


Constant::~Constant()
{
}
