#pragma once

//从ATL的base64库整理而来
//整理的目的是提供一个仅基于标准c++的base64库，提高代码的可移植性
#include <string>

namespace Crypt
{
    class Base64
    {
    public:
        enum Base64_Flag
        {
            ATL_BASE64_FLAG_NONE = 0, // 0000 0000  标准的编码方式（RFC2045）
            ATL_BASE64_FLAG_NOPAD = 1, // 0000 0001  没有补全（ = ）
            ATL_BASE64_FLAG_NOCRLF = 2 // 0000 0010   没有换行 ( crlf )
        };

        //计算编码后的长度
        static int EncodeGetRequiredLength(int nSrcLen, Base64_Flag dwFlags = ATL_BASE64_FLAG_NONE)
        {
            int nRet = static_cast<int>((nSrcLen << 2) / 3);
            if ((dwFlags & ATL_BASE64_FLAG_NOPAD) == 0)// 需要补全 =
                nRet += nSrcLen % 3;
            int nOnLastLine = nRet % 76;
            if (nOnLastLine)
                if ((nOnLastLine & 0x03)) //nOnLastLine % 4
                    nRet += 4 - (nOnLastLine & 0x03);
            if ((dwFlags & ATL_BASE64_FLAG_NOCRLF) == 0) //需要换行符
                nRet += ((nRet / 76 + 1) << 1);//(nRet / 76 + 1) * 2, 回车换行个数（+1是为了取最多值），根据rfc2045，编码后一行最多76个字符（不含换行）
            return nRet;
        }

        inline static int DecodeGetRequiredLength(int nSrcLen)
        {
            return ((nSrcLen * 3) >> 2) + 2; //(nSrcLen*3)/4+2
        }

        //pbSrcData  要编码的原始数据
        //nSrcLen  原始数据长度
        //szDest    编码后的输出
        //pnEncodeRealLen       编码后的实际长度
        static bool Encode(const unsigned char* pbSrcData, int nSrcLen, char* szDest, int* pnEncodeRealLen, Base64_Flag dwFlags = ATL_BASE64_FLAG_NONE)
        {
            static const char* s_chBase64EncodingTable = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            if (!pbSrcData || !szDest || !pnEncodeRealLen)
                return false;
            int nWritten = 0; //编码后的长度，EncodeGetRequiredLength获得的长度通常大于实际长度
            int nLen1 = (nSrcLen / 3) << 2; // (nSrcLen/3)*4
            int nLen2 = nLen1 / 76;  //RFC2045 base64编码后，每行最多76个字符
            int nLen3 = 19;
            for (int i = 0; i <= nLen2; i++)
            {
                if (i == nLen2)
                    nLen3 = (nLen1 % 76) / 4;
                for (int j = 0; j < nLen3; j++)
                {
                    unsigned long dwCurr = 0;
                    for (int n = 0; n < 3; n++)
                    {
                        dwCurr |= *pbSrcData++;
                        dwCurr <<= 8;
                    }
                    for (int k = 0; k < 4; k++)
                    {
                        unsigned char b = (unsigned char)(dwCurr >> 26);
                        *szDest++ = s_chBase64EncodingTable[b];
                        dwCurr <<= 6;
                    }
                }
                nWritten += nLen3 * 4;
                if ((dwFlags & ATL_BASE64_FLAG_NOCRLF) == 0)
                {
                    *szDest++ = '\r';
                    *szDest++ = '\n';
                    nWritten += 2;
                }
            }
            if (nWritten && (dwFlags & ATL_BASE64_FLAG_NOCRLF) == 0)
            {
                szDest -= 2;
                nWritten -= 2;
            }
            nLen2 = (nSrcLen % 3) ? (nSrcLen % 3 + 1) : 0;
            if (nLen2)
            {
                unsigned long dwCurr = 0;
                for (int n = 0; n < 3; n++)
                {
                    if (n < (nSrcLen % 3))
                        dwCurr |= *pbSrcData++;
                    dwCurr <<= 8;
                }
                for (int k = 0; k < nLen2; k++)
                {
                    unsigned char b = (unsigned char)(dwCurr >> 26);
                    *szDest++ = s_chBase64EncodingTable[b];
                    dwCurr <<= 6;
                }
                nWritten += nLen2;
                if ((dwFlags & ATL_BASE64_FLAG_NOPAD) == 0)
                {
                    nLen3 = nLen2 ? 4 - nLen2 : 0;
                    for (int j = 0; j < nLen3; j++)
                        *szDest++ = '=';
                    nWritten += nLen3;
                }
            }
            *pnEncodeRealLen = nWritten;
            return true;
        }

        //szSrc  要解码的数据
        //nSrcLen  要解码数据的长度
        //pbDest   解码后的输出
        //pnDecodeRealLen  解码后的字节长度
        static bool Decode(const char* szSrc, int nSrcLen, unsigned char* pbDest, int* pnDecodeRealLen)
        {
            // walk the source buffer
            // each four character sequence is converted to 3 bytes
            // CRLFs and =, and any characters not in the encoding table
            // are skiped

            // returns -1 if the character is invalid
            // or should be skipped
            // otherwise, returns the 6-bit code for the character
            // from the encoding table
            static const int base64_decode_table[] =
            {
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, 62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1,
                -1, -1, -1, -1, -1,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
                15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1, -1, 26, 27, 28,
                29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
                49, 50, 51, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
            };
            if (szSrc == 0 || pnDecodeRealLen == 0)
                return false;
            const char* szSrcEnd = szSrc + nSrcLen;
            int nWritten = 0;

            while (szSrc < szSrcEnd && (*szSrc) != 0)
            {
                unsigned long dwCurr = 0;
                int i;
                int nBits = 0;
                for (i = 0; i < 4; i++)
                {
                    if (szSrc >= szSrcEnd)
                        break;
                    int nCh = base64_decode_table[(unsigned char)(*szSrc)]; //强转为unsigned char，防止运行时抛出异常
                    szSrc++;
                    if (nCh == -1)
                    {
                        // skip this char
                        i--;
                        continue;
                    }
                    dwCurr <<= 6;
                    dwCurr |= nCh;
                    nBits += 6;
                }

                // dwCurr has the 3 bytes to write to the output buffer
                // left to right
                dwCurr <<= 24 - nBits;
                for (i = 0; i < nBits / 8; i++)
                {
                    *pbDest = (unsigned char)((dwCurr & 0x00ff0000) >> 16);
                    pbDest++;
                    dwCurr <<= 8;
                    nWritten++;
                }
            }
            *pnDecodeRealLen = nWritten;
            return true;
        }
        static bool Encode(const std::string& in, std::string& out, Base64_Flag dwFlags = ATL_BASE64_FLAG_NONE)
        {
            int len = EncodeGetRequiredLength(static_cast<int>(in.size()), dwFlags);
            out.assign(len, 0);
            bool ret = Encode((const unsigned char*)in.c_str(), static_cast<int>(in.size()), (char*)out.c_str(), &len, dwFlags);
            out.resize(static_cast<size_t>(len));
            return ret;
        }
        static bool Encode(const unsigned char* in, int in_len, std::string& out, Base64_Flag dwFlags = ATL_BASE64_FLAG_NONE)
        {
            int len = EncodeGetRequiredLength(in_len, dwFlags);
            out.assign(len, 0);
            bool ret = Encode(in, in_len, (char*)out.c_str(), &len, dwFlags);
            out.resize(static_cast<size_t>(len));
            return ret;
        }
        static bool Decode(const std::string& in, std::string& out)
        {
            out.assign(in.size(), 0);
            int len = static_cast<int>(out.size());
            bool ret = Decode(in.c_str(), static_cast<int>(in.size()), (unsigned char*)out.c_str(), &len);
            out.resize(static_cast<size_t>(len));
            return ret;
        }
        static bool Decode(const char* in, int in_len, std::string& out)
        {
            out.assign(static_cast<size_t>(in_len), 0);
            int outlen = static_cast<int>(out.size());
            bool ret = Decode(in, in_len, (unsigned char*)out.c_str(), &outlen);
            out.resize(static_cast<size_t>(outlen));
            return ret;
        }
    };
}
