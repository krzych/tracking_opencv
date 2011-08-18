#pragma once
class Error
{
private:
	int m_iErrorCode;
	const char* const m_csMessage;
	static bool m_bIsError;
public:
	Error(int,const char*);
	~Error(void);

	inline bool isError() {return m_bIsError;}
	inline int getErrorCode() {return m_iErrorCode;}
	inline const char* getMessage() {return m_csMessage;}

};

