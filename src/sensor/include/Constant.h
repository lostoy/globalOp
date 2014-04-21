#pragma once

class Constant
{
public:
	Constant();
	~Constant();

	/* The length of point in bytes in .bin files produced from Depth2Ply */
	static const int POINT_XYZRGBA_LENGTH_IN_BYTES;

	/* The length of point in bytes in .bin files used by Xuetong Sun in preview */
	static const int POINT_XYZRGBA_LENGTH_IN_BYTES_XT;

	/* Bad point for invalid depth point */
	static const float BAD_POINT_VALUE;

	static const size_t DEFAULT_BIN_FILE_SIZE;
};

