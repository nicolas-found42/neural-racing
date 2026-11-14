// Controlled local harness for original CORE routines using the upstream Play! EE engine.
// This file is analyst-authored; the linked emulator retains its original licence.
#include "MIPS.h"
#include "ee/EeExecutor.h"
#include "ee/MA_EE.h"
#include "COP_FPU.h"
#include "COP_SCU.h"
#include "ee/COP_VU.h"
#include "FpUtils.h"
#include "Ps2Const.h"
#include "AlignedAlloc.h"
#include <cstdio>
#include <cstring>
#include <cfenv>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <sys/mman.h>

constexpr uint32 RAM_SIZE=PS2::EE_RAM_SIZE;
constexpr uint32 RETURN_STOP=0x01fff000;
// Play! breakpoints stop at containing basic-block entry. Split only the JIT
// block at our requested boundary so it stops before that exact instruction.
// Guest opcodes and their order are unchanged.
class BoundedEeExecutor : public CEeExecutor {
public:
    using CEeExecutor::CEeExecutor;
    uint32 stopAddress=RETURN_STOP;
protected:
    void PartitionFunction(uint32 start) override {
        if(start<stopAddress && stopAddress-start<=MAX_BLOCK_SIZE){
            bool branchBeforeStop=false;
            for(uint32 a=start;a<stopAddress;a+=4){
                uint32 op=m_context.m_pMemoryMap->GetInstruction(a);
                if(m_context.m_pArch->IsInstructionBranch(&m_context,a,op)!=MIPS_BRANCH_NONE){branchBeforeStop=true;break;}
            }
            if(!branchBeforeStop){CreateBlock(start,stopAddress-4);SetupBlockLinks(start,stopAddress-4,MIPS_INVALID_PC);return;}
        }
        CEeExecutor::PartitionFunction(start);
    }
};
uint32 U32(std::istream& s){uint32 x=0;s.read((char*)&x,4);if(!s)throw std::runtime_error("truncated fixture");return x;}
uint64 U64(std::istream& s){uint64 x=0;s.read((char*)&x,8);if(!s)throw std::runtime_error("truncated fixture");return x;}
uint32 At32(const std::vector<uint8>& b,size_t p){uint32 x;memcpy(&x,b.data()+p,4);return x;}
uint16 At16(const std::vector<uint8>& b,size_t p){uint16 x;memcpy(&x,b.data()+p,2);return x;}
std::vector<uint8> ReadFile(const char* p){std::ifstream f(p,std::ios::binary);if(!f)throw std::runtime_error("cannot open input");return {std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>()};}

int main(int argc,char** argv){try{
    if(argc<3 || argc>5)throw std::runtime_error("usage: gt4-runner CORE.ps2.elf fixtures.bin [nearest-multiply-sensitivity|ieee-nearest-sensitivity] [--cycle-budget=1000000]");
    uint32 cycleBudget=1000000;bool nearest=false;bool ieeeNearest=false;
    for(int i=3;i<argc;i++){
        std::string option=argv[i];
        if(option.rfind("--cycle-budget=",0)==0){
            auto value=std::stoul(option.substr(15));
            if(value<1000 || value>100000000)throw std::runtime_error("cycle budget must be 1000..100000000");
            cycleBudget=static_cast<uint32>(value);
        }else if(option=="nearest-multiply-sensitivity" || option=="ieee-nearest-sensitivity"){
            nearest=true;ieeeNearest=option=="ieee-nearest-sensitivity";
        }else throw std::runtime_error("unknown host runner option");
    }
    auto elf=ReadFile(argv[1]);std::vector<uint8> baseline(RAM_SIZE,0);
    if(At32(elf,0)!=0x464c457f||elf[4]!=1||elf[5]!=1)throw std::runtime_error("requires ELF32 little-endian");
    for(uint32 i=0;i<At16(elf,44);i++){
        size_t ph=At32(elf,28)+i*At16(elf,42);if(At32(elf,ph)!=1)continue;
        uint32 off=At32(elf,ph+4),va=At32(elf,ph+8),n=At32(elf,ph+16);
        if(va+n>RAM_SIZE||off+n>elf.size())throw std::runtime_error("invalid segment");
        memcpy(baseline.data()+va,elf.data()+off,n);
    }
    auto ram=(uint8*)framework_aligned_alloc(RAM_SIZE,framework_getpagesize());
    auto scratch=(uint8*)framework_aligned_alloc(0x4000,0x10);
    auto vu=(uint8*)framework_aligned_alloc(0x4000,0x10);
    memset(scratch,0,0x4000);memset(vu,0,0x4000);
    CMIPS cpu(MEMORYMAP_ENDIAN_LSBF,true);CMA_EE architecture;
    CCOP_FPU fpu(MIPS_REGSIZE_64);CCOP_SCU scu(MIPS_REGSIZE_64);CCOP_VU cop2(MIPS_REGSIZE_64);
    cpu.m_pArch=&architecture;cpu.m_pCOP[0]=&scu;cpu.m_pCOP[1]=&fpu;cpu.m_pCOP[2]=&cop2;
    cpu.m_pAddrTranslator=CMIPS::TranslateAddress64;cpu.m_vuMem=vu;
    cpu.m_pMemoryMap->InsertReadMap(0,RAM_SIZE-1,ram,0);cpu.m_pMemoryMap->InsertWriteMap(0,RAM_SIZE-1,ram,0);cpu.m_pMemoryMap->InsertInstructionMap(0,RAM_SIZE-1,ram,0);
    cpu.m_pMemoryMap->InsertReadMap(0x70000000,0x70003fff,scratch,1);cpu.m_pMemoryMap->InsertWriteMap(0x70000000,0x70003fff,scratch,1);
    cpu.MapPages(0,RAM_SIZE,ram);cpu.MapPages(0x70000000,0x4000,scratch);cpu.MapPages(0x80000000,RAM_SIZE,ram);
    cpu.m_executor=std::make_unique<BoundedEeExecutor>(cpu,ram);auto executor=static_cast<BoundedEeExecutor*>(cpu.m_executor.get());
    CEeExecutor::BlockFpUseAccurateAddSubSet accurate;
    for(uint32 a=0x100000;a<0x617a14;a+=4)accurate.insert(a);
    if(!ieeeNearest) executor->SetBlockFpUseAccurateAddSub(std::move(accurate));
    std::fesetround(nearest ? FE_TONEAREST : FE_TOWARDZERO);FpUtils::SetDenormalHandlingMode();
    std::ifstream input(argv[2],std::ios::binary);char magic[8];input.read(magic,8);if(memcmp(magic,"GT4RUN1",7))throw std::runtime_error("bad fixture magic");
    uint32 count=U32(input);
    for(uint32 idx=0;idx<count;idx++){
        executor->Reset();cpu.Reset();cpu.m_breakpoints.clear();if(mprotect(ram,RAM_SIZE,PROT_READ|PROT_WRITE)!=0)throw std::runtime_error("unprotect fixture RAM failed");memcpy(ram,baseline.data(),RAM_SIZE);memset(scratch,0,0x4000);memset(vu,0,0x4000);
        uint32 entry=U32(input),stop=U32(input);executor->stopAddress=stop;cpu.m_State.nPC=entry;cpu.m_State.nGPR[CMIPS::SP].nD0=0x01ff0000;cpu.m_State.nGPR[CMIPS::RA].nD0=RETURN_STOP;cpu.m_State.nGPR[CMIPS::GP].nD0=0x6dddf0;
        for(uint32 n=U32(input);n;n--){uint32 reg=U32(input);uint64 v=U64(input);if(reg>=32)throw std::runtime_error("bad gpr");cpu.m_State.nGPR[reg].nD0=v;}
        for(uint32 n=U32(input);n;n--){uint32 reg=U32(input),v=U32(input);if(reg>=32)throw std::runtime_error("bad fpr");cpu.m_State.nCOP1[reg]=v;}
        for(uint32 n=U32(input);n;n--){uint32 addr=U32(input),size=U32(input);if(addr+size>RAM_SIZE)throw std::runtime_error("bad memory patch");input.read((char*)ram+addr,size);}
        std::vector<uint32> reads;for(uint32 n=U32(input);n;n--){uint32 a=U32(input);if(a+4>RAM_SIZE && !(a>=0xffff0000 && a<0xffff0280))throw std::runtime_error("bad read");reads.push_back(a);}
        cpu.m_breakpoints.insert(stop);uint32 quota=0;
        while(cpu.m_State.nPC!=stop){
            if(quota>=cycleBudget)throw std::runtime_error("guest cycle budget exceeded");
            executor->Execute(1000);quota+=1000;
            if(executor->MustBreak())break;
            if(cpu.m_State.nHasException)throw std::runtime_error("unexpected guest exception "+std::to_string(cpu.m_State.nHasException));
        }
        if(cpu.m_State.nPC!=stop)throw std::runtime_error("wrong stopping PC idx="+std::to_string(idx)+" actual="+std::to_string(cpu.m_State.nPC)+" expected="+std::to_string(stop));
        printf("%u %08x %08x %08x %u",idx,cpu.m_State.nPC,cpu.m_State.nGPR[CMIPS::V0].nV0,cpu.m_State.nCOP1[0],quota);
        for(auto a:reads){uint32 v;if(a>=0xffff0200)v=cpu.m_State.nGPR[(a-0xffff0200)/4].nV0;else if(a==0xffff0100)v=cpu.m_State.nCOP1A;else if(a>=0xffff0000)v=cpu.m_State.nCOP1[(a-0xffff0000)/4];else memcpy(&v,ram+a,4);printf(" %08x",v);}printf("\n");fflush(stdout);
    }
    executor->Reset();cpu.m_executor.reset();framework_aligned_free(ram);framework_aligned_free(scratch);framework_aligned_free(vu);return 0;
}catch(const std::exception& e){fprintf(stderr,"gt4-runner error: %s\n",e.what());return 1;}}
