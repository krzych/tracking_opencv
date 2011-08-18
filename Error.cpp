#include "Error.h"


Error::Error(int ErrorCode, const char* const ErrorMessage)
	:m_csMessage(ErrorMessage), m_iErrorCode(ErrorCode)
{
}


Error::~Error(void)
{
}
