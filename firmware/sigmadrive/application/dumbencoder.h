#ifndef _DUMBENCODER_H_
#define _DUMBENCODER_H_

#include "iencoder.h"

class DumbEncoder : public IEncoder {
public:
    DumbEncoder() = default;
    virtual ~DumbEncoder() { }

    virtual bool Initialize() override { return true; }
    virtual void ResetPosition() override { }
    virtual uint32_t GetResolutionBits() override { return 16; }
    virtual uint32_t GetRevolutionBits() override { return 16; }
    virtual uint64_t GetPosition() override { return 0; }
    virtual uint32_t GetIndexPosition() override { return 0; }
    virtual uint32_t GetLastError() override { return 0; }
    virtual bool Update() override { return true; }
};

#endif /* _DUMBENCODER_H_ */
