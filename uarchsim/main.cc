// See LICENSE for license details.

#include "sim.h"
#include "htif.h"
#include "cachesim.h"
#include "extension.h"
#include <dlfcn.h>
#include <fesvr/option_parser.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include "debug.h"
#include "parameters.h"
#include <signal.h>

static void help()
{
  fprintf(stderr, "usage: micros [host options] <target program> [target options]\n");
  fprintf(stderr, "Host Options:\n");
  fprintf(stderr, "  -c<gz_chkpt_file>  Start simulation from a .gz checkpoint file.\n");
  fprintf(stderr, "  -d                 Interactive debug mode\n");
  fprintf(stderr, "  -e<n>              End simulation after <n> instructions have been committed by microarchitectural simulation\n");
  fprintf(stderr, "  -g                 Track histogram of PCs\n");
  fprintf(stderr, "  -h                 Print this help message\n");
  fprintf(stderr, "  -l<n>              Enable logging after <n> commits if compiled with support\n");
  fprintf(stderr, "  -m<n>              Provide <n> MB of target memory\n");
  fprintf(stderr, "  -p<n>              Simulate <n> processors\n");
  fprintf(stderr, "  -s<n>              Fast skip <n> instructions before microarchitectural simulation\n");
  fprintf(stderr, "  --perf=<pbp>,<pdc>,<pic>,<ptc>\tEach of pbp (perf. branch pred.), pdc (perf. D$), pic (perf. I$), and ptc (perf. T$), are 0 or 1\n");
  fprintf(stderr, "  --cp=<n>           <n> branch checkpoints for mispredict recovery\n");

  fprintf(stderr, "  --bq=<n>           Branch queue (all branches b/w fetch and retire) has <n> entries\n");
  fprintf(stderr, "  --btbentries=<n>   BTB has a total of <n> entries\n");
  fprintf(stderr, "  --btbassoc=<n>     BTB has a set-associativity of <n>\n");
  fprintf(stderr, "  --ras=<n>          RAS has <n> entries\n");
  fprintf(stderr, "  --mbp=<n>          The conditional branch predictor (whether real or perfect) can predict a maximum of <n> conditional branches per cycle\n");
  fprintf(stderr, "  --cbpPC=<n>        The gshare-indexed conditional branch predictor uses <n> bits of PC\n");
  fprintf(stderr, "  --cbpBHR=<n>       The gshare-indexed conditional branch predictor uses <n> bits of BHR\n");
  fprintf(stderr, "  --ibpPC=<n>        The gshare-indexed indirect branch predictor uses <n> bits of PC\n");
  fprintf(stderr, "  --ibpBHR=<n>       The gshare-indexed indirect branch predictor uses <n> bits of BHR\n");
  fprintf(stderr, "  -t                 Enable trace cache\n");

  fprintf(stderr, "  --fq=<n>           Fetch queue has <n> entries\n");
  fprintf(stderr, "  --al=<n>           Active List has <n> entries\n");
  fprintf(stderr, "  --prf=<n>          Physical Register File has <n> physical registers\n");
  fprintf(stderr, "  --iq=<n>           Issue Queue has <n> entries\n");
  fprintf(stderr, "  --iqnp=<n>         Issue Queue has <n> partitions for round-robin partition-based priority adjustment\n");
  fprintf(stderr, "  -a                 Enable pre-steering in dispatch stage (override dynamic lane steering at issue stage)\n");
  fprintf(stderr, "  -b                 Enable ideal age-based scheduling (override position-based scheduling)\n");
  fprintf(stderr, "  --lsq=<n>          Load/Store Queue has <n> entries\n");
  fprintf(stderr, "  --disambig=<mdp_model>,<mdp_ctr_max>\t<mdp_model>: 0 (always pred. conflict), 1 (always pred. no conflict), 2 (MDP-sticky), 3 (MDP-ctr), 4 (oracle). <mdp_ctr_max>: max counter value for MDP-ctr.\n");
  fprintf(stderr, "  --fw=<n>           <n> wide fetch\n");
  fprintf(stderr, "  --dw=<n>           <n> wide dispatch\n");
  fprintf(stderr, "  --iw=<n>           <n> wide issue / <n> execution lanes\n");
  fprintf(stderr, "  --rw=<n>           <n> wide retire\n");
  fprintf(stderr, "  --phase=<n>        Phase interval is <n>\n");
  fprintf(stderr, "  --lane=<B>:<L>:<S>:<C>:<LFP>:<FP>:<MTF>\tEach of <X> is a bit vector indicating which lanes support that instruction type.\n");
  fprintf(stderr, "  --lat=<B>:<L>:<S>:<C>:<LFP>:<FP>:<MTF>\tEach of <X> is an unsigned integer indicating the latency of that instruction type.\n");
  fprintf(stderr, "  -u                 Shortcut to configure universal lanes. Equivalent to: --lane=0xffff:0xffff:0xffff:0xffff:0xffff:0xffff:0xffff --lat=1:1:1:1:1:1:1\n");
  fprintf(stderr, "  --L2L3exist=a,b\tEnable (a=1) or disable (a=0) the L2 cache. Enable (b=1) or disable (b=0) the L3 cache.\n");
  fprintf(stderr, "  --IC=<SIZE>:<ASSOC>:<BLOCKSIZE>:<#MHSR>\tConfigure L1 I$. Derived # sets must be power-of-2. Block size must be power-of-2.\n");
  fprintf(stderr, "  --DC=<SIZE>:<ASSOC>:<BLOCKSIZE>:<#MHSR>\tConfigure L1 D$. Derived # sets must be power-of-2. Block size must be power-of-2.\n");
  fprintf(stderr, "  --L2=<SIZE>:<ASSOC>:<BLOCKSIZE>:<#MHSR>:<HITTIME>\tConfigure L2 $. Derived # sets must be power-of-2. Block size must be power-of-2.\n");
  fprintf(stderr, "  --L3=<SIZE>:<ASSOC>:<BLOCKSIZE>:<#MHSR>:<HITTIME>\tConfigure L3 $. Derived # sets must be power-of-2. Block size must be power-of-2.\n");
  fprintf(stderr, "  --MEMLAT=<latency>\tConfigure a fixed miss penalty for a miss in the LLC.\n");
  exit(1);
}

/* execution start time */
time_t start_time;


static void sim_stats(FILE* stream)
{
}  

static void exit_now(int sigtype)
{
  sim_stats(stderr);
  //for (unsigned int i = 0; i < NumThreads; i++)
  //   THREAD[i]->mem_stats(stderr);
  exit(1);
}

static void sim_config(FILE* stream)
{
}

static void mem_config(FILE* stream)
{
}

static void set_lane_matrix(const char* config)
{
  //const char* bp = strchr(config, ':');
  //if (!bp++) help();
  //const char* lp = strchr(bp, ':');
  //if (!lp++) help();
  //const char* sp = strchr(lp, ':');
  //if (!sp++) help();
  //const char* cp = strchr(sp, ':');
  //if (!cp++) help();
  //const char* fp = strchr(cp, ':');
  //if (!fp++) help();
  //const char* lfp = strchr(fp, ':');
  //if (!lfp++) help();
  //const char* mtfp = strchr(lfp, ':');
  //if (!mtfp++) help();

  if(std::count(config, config+std::strlen(config), ':') != 6)
    help();

  char *pEnd;
  FU_LANE_MATRIX[0] = strtol(config ,&pEnd,16)   /*     BR: 0000 0010 */;
  pEnd++;
  FU_LANE_MATRIX[1] = strtol(pEnd   ,&pEnd,16)   /*     LS: 0001 0001 */;
  pEnd++;
  FU_LANE_MATRIX[2] = strtol(pEnd   ,&pEnd,16)   /*  ALU_S: 0000 1110 */;
  pEnd++;
  FU_LANE_MATRIX[3] = strtol(pEnd   ,&pEnd,16)   /*  ALU_C: 0000 0010 */;
  pEnd++;
  FU_LANE_MATRIX[4] = strtol(pEnd   ,&pEnd,16)   /*  LS_FP: 0001 0001 */;
  pEnd++;
  FU_LANE_MATRIX[5] = strtol(pEnd   ,&pEnd,16)   /* ALU_FP: 0000 0110 */;
  pEnd++;
  FU_LANE_MATRIX[6] = strtol(pEnd   ,NULL ,16)   /*    MTF: 0000 0010 */;
}

static void set_lane_latencies(const char* config) {
   if (sscanf(config, "%u:%u:%u:%u:%u:%u:%u", &(FU_LAT[0]), &(FU_LAT[1]), &(FU_LAT[2]), &(FU_LAT[3]), &(FU_LAT[4]), &(FU_LAT[5]), &(FU_LAT[6])) != 7) {
      fprintf(stderr, "Incorrect usage of --lat=<B>:<L>:<S>:<C>:<LFP>:<FP>:<MTF>\n");
      fprintf(stderr, "...where each of <X> is an unsigned integer indicating the latency of that instruction type.\n");
   }
}

static void set_perfect_flags(const char* config) {
   uint64_t pbp, pdc, pic, ptc;
   if (sscanf(config, "%lu,%lu,%lu,%lu", &pbp, &pdc, &pic, &ptc) != 4) {
      fprintf(stderr, "Incorrect usage of --perf=<pbp>,<pdc>,<pic>,<ptc>\n");
      fprintf(stderr, "...where pbp (perfect branch prediction), pdc (perfect D$), pic (perfect I$), and ptc (perfect T$) are each 0 or 1.\n");
      exit(-1);
   }
   else {
      PERFECT_BRANCH_PRED = (pbp ? true : false);
      PERFECT_DCACHE = (pdc ? true : false);
      PERFECT_ICACHE = (pic ? true : false);
      PERFECT_TRACE_CACHE = (ptc ? true : false);
   }
}

static void set_disambig_flags(const char* config) {
   uint64_t mdp_model, mdp_ctr_max;
   if (sscanf(config, "%lu,%lu", &mdp_model, &mdp_ctr_max) != 2) {
      fprintf(stderr, "Incorrect usage:\n");
      fprintf(stderr, "--disambig=<mdp_model>,<mdp_ctr_max>\t<mdp_model>: 0 (always pred. conflict), 1 (always pred. no conflict), 2 (MDP-sticky), 3 (MDP-ctr), 4 (oracle). <mdp_ctr_max>: max counter value for MDP-ctr.\n");
      exit(-1);
   }
   else {
      ORACLE_DISAMBIG = false;
      SPEC_DISAMBIG = false;
      MEM_DEP_PRED = false;
      MDP_STICKY = false;
      MDP_MAX = mdp_ctr_max;
      switch (mdp_model) {
         case 0:
	    // Always predict conflict.
	    break;

         case 1:
	    // Always predict no conflict.
	    SPEC_DISAMBIG = true;
	    break;

         case 2:
	    // MDP-sticky.
	    SPEC_DISAMBIG = true;
	    MEM_DEP_PRED = true;
	    MDP_STICKY = true;
	    MDP_MAX = 1;
	    break;

         case 3:
	    // MDP-ctr.
	    SPEC_DISAMBIG = true;
	    MEM_DEP_PRED = true;

            if (MDP_MAX == 0) {
               fprintf(stderr, "Incorrect usage:\n");
               fprintf(stderr, "--disambig=<mdp_model>,<mdp_ctr_max>\t<mdp_model>: 0 (always pred. conflict), 1 (always pred. no conflict), 2 (MDP-sticky), 3 (MDP-ctr), 4 (oracle). <mdp_ctr_max>: max counter value for MDP-ctr.\n");
               fprintf(stderr, "<mdp_ctr_max> (%u) must be greater than 0.\n", MDP_MAX);
               exit(-1);
            }
            break;

         case 4:
	    // Oracle.
            ORACLE_DISAMBIG = true;
            break;

         default:
            fprintf(stderr, "Incorrect usage:\n");
            fprintf(stderr, "--disambig=<mdp_model>,<mdp_ctr_max>\t<mdp_model>: 0 (always pred. conflict), 1 (always pred. no conflict), 2 (MDP-sticky), 3 (MDP-ctr), 4 (oracle). <mdp_ctr_max>: max counter value for MDP-ctr.\n");
            fprintf(stderr, "<mdp_model> (%lu) must be 0 to 4.\n", mdp_model);
            exit(-1);
	    break;
      }
   }
}

static void config_IC(const char* config) {
   unsigned int temp_size, temp_blocksize;
   if (sscanf(config, "%u:%u:%u:%u", &temp_size, &L1_IC_ASSOC, &temp_blocksize, &L1_IC_NUM_MHSRs) != 4) {
      fprintf(stderr, "Incorrect usage of --IC=<SIZE>:<ASSOC>:<BLOCKSIZE>:<#MHSR>.\n");
      exit(-1);
   }
   else {
      L1_IC_MISS_SRV_PORTS = L1_IC_NUM_MHSRs;
      if (!IsPow2(temp_blocksize)) {
         fprintf(stderr, "--IC: IC block size (%u) must be a power-of-2.\n", temp_blocksize);
         exit(-1);
      }
      else {
         L1_IC_LINE_SIZE = (unsigned int) log2((double)temp_blocksize);
      }
      L1_IC_SETS = (temp_size/(L1_IC_ASSOC*temp_blocksize));
      if (!IsPow2(L1_IC_SETS)) {
         fprintf(stderr, "--IC: IC derived # sets (%u) must be a power-of-2.\n", L1_IC_SETS);
         exit(-1);
      }
   }
}

static void config_DC(const char* config) {
   unsigned int temp_size, temp_blocksize;
   if (sscanf(config, "%u:%u:%u:%u", &temp_size, &L1_DC_ASSOC, &temp_blocksize, &L1_DC_NUM_MHSRs) != 4) {
      fprintf(stderr, "Incorrect usage of --DC=<SIZE>:<ASSOC>:<BLOCKSIZE>:<#MHSR>.\n");
      exit(-1);
   }
   else {
      L1_DC_MISS_SRV_PORTS = L1_DC_NUM_MHSRs;
      if (!IsPow2(temp_blocksize)) {
         fprintf(stderr, "--DC: DC block size (%u) must be a power-of-2.\n", temp_blocksize);
         exit(-1);
      }
      else {
         L1_DC_LINE_SIZE = (unsigned int) log2((double)temp_blocksize);
      }
      L1_DC_SETS = (temp_size/(L1_DC_ASSOC*temp_blocksize));
      if (!IsPow2(L1_DC_SETS)) {
         fprintf(stderr, "--DC: DC derived # sets (%u) must be a power-of-2.\n", L1_DC_SETS);
         exit(-1);
      }
   }
}

static void config_L2(const char* config) {
   unsigned int temp_size, temp_blocksize;
   if (sscanf(config, "%u:%u:%u:%u:%u", &temp_size, &L2_ASSOC, &temp_blocksize, &L2_NUM_MHSRs, &L2_HIT_LATENCY) != 5) {
      fprintf(stderr, "Incorrect usage of --L2=<SIZE>:<ASSOC>:<BLOCKSIZE>:<#MHSR>:<HITTIME>.\n");
      exit(-1);
   }
   else {
      L2_MISS_SRV_PORTS = L2_NUM_MHSRs;
      if (!IsPow2(temp_blocksize)) {
         fprintf(stderr, "--L2: L2 block size (%u) must be a power-of-2.\n", temp_blocksize);
         exit(-1);
      }
      else {
         L2_LINE_SIZE = (unsigned int) log2((double)temp_blocksize);
      }
      L2_SETS = (temp_size/(L2_ASSOC*temp_blocksize));
      if (!IsPow2(L2_SETS)) {
         fprintf(stderr, "--L2: L2 derived # sets (%u) must be a power-of-2.\n", L2_SETS);
         exit(-1);
      }
   }
}

static void config_L3(const char* config) {
   unsigned int temp_size, temp_blocksize;
   if (sscanf(config, "%u:%u:%u:%u:%u", &temp_size, &L3_ASSOC, &temp_blocksize, &L3_NUM_MHSRs, &L3_HIT_LATENCY) != 5) {
      fprintf(stderr, "Incorrect usage of --L3=<SIZE>:<ASSOC>:<BLOCKSIZE>:<#MHSR>:<HITTIME>.\n");
      exit(-1);
   }
   else {
      L3_MISS_SRV_PORTS = L3_NUM_MHSRs;
      if (!IsPow2(temp_blocksize)) {
         fprintf(stderr, "--L3: L3 block size (%u) must be a power-of-2.\n", temp_blocksize);
         exit(-1);
      }
      else {
         L3_LINE_SIZE = (unsigned int) log2((double)temp_blocksize);
      }
      L3_SETS = (temp_size/(L3_ASSOC*temp_blocksize));
      if (!IsPow2(L3_SETS)) {
         fprintf(stderr, "--L3: L3 derived # sets (%u) must be a power-of-2.\n", L3_SETS);
         exit(-1);
      }
   }
}

static void config_L2L3present(const char* config) {
   int a, b;
   if (sscanf(config, "%d,%d", &a, &b) != 2) {
      fprintf(stderr, "Incorrect usage: --L2L3exist=a,b\tEnable (a=1) or disable (a=0) the L2 cache. Enable (b=1) or disable (b=0) the L3 cache.\n");
      exit(-1);
   }
   else {
      L2_PRESENT = (a ? true : false);
      L3_PRESENT = (L2_PRESENT && (b ? true : false));
   }
}

/* exit when this becomes non-zero */
//int sim_exit_now = FALSE;
// Should be global variables for access from all DPI functions
debug_buffer_t* DB;
sim_t*  s_isa;
sim_t*  s_micro;

static void endSimulation(int signal)
{
  //*** Must delete the simulator instances in order to dump stats ***
  // Stats are dumped in the destructor for the processor instances.
  delete s_isa;
  delete s_micro;
}  



int main(int argc, char** argv)
{
  bool debug = false;
  bool histogram = false;
  //bool simpoint = false;
  //size_t simpoint_interval = 100000000;
  //bool checkpoint = false;
  //size_t checkpoint_skip_amt = 0;
  size_t nprocs = 1;
  size_t mem_mb = 0;
  size_t skip_amt = 0;   /////////////
  bool skip_enable = false;   /////////////

  std::string checkpoint_file = "";

  option_parser_t parser;
  parser.help(&help);
  parser.option('h', 0, 0, [&](const char* s){help();});
  parser.option('d', 0, 0, [&](const char* s){debug = true;});
  parser.option('g', 0, 0, [&](const char* s){histogram = true;});
  parser.option('l', 0, 1, [&](const char* s){logging_on_at = atoll(s);});
  parser.option('p', 0, 1, [&](const char* s){nprocs = atoi(s);});
  parser.option('m', 0, 1, [&](const char* s){mem_mb = atoi(s);});
  parser.option('s', 0, 1, [&](const char* s){skip_amt = atoll(s); skip_enable = true;});
  parser.option('e', 0, 1, [&](const char* s){stop_amt = atoll(s); use_stop_amt = true;});
  parser.option('c', 0, 1, [&](const char* s){checkpoint_file = s;});
  parser.option(0, "IC", 1, [&](const char* s){config_IC(s);});
  parser.option(0, "DC", 1, [&](const char* s){config_DC(s);});
  parser.option(0, "L2", 1, [&](const char* s){config_L2(s);});
  parser.option(0, "L3", 1, [&](const char* s){config_L3(s);});
  parser.option(0, "L2L3exist", 1, [&](const char* s){config_L2L3present(s);});
  parser.option(0, "MEMLAT", 1, [&](const char* s){L1_IC_MISS_LATENCY = L1_DC_MISS_LATENCY = L2_MISS_LATENCY = atoi(s);});
  parser.option(0, "perf", 1, [&](const char* s){set_perfect_flags(s);});
  parser.option(0, "cp"  , 1, [&](const char* s){NUM_CHECKPOINTS = atoi(s);});

  parser.option(0, "bq", 1, [&](const char* s){BQ_SIZE = atoi(s); AUTO_BQ_SIZE = false;});
  parser.option(0, "btbentries", 1, [&](const char* s){BTB_ENTRIES = atoi(s);});
  parser.option(0, "btbassoc", 1, [&](const char* s){BTB_ASSOC = atoi(s);});
  parser.option(0, "ras", 1, [&](const char* s){RAS_SIZE = atoi(s);});
  parser.option(0, "mbp", 1, [&](const char* s){COND_BRANCH_PRED_PER_CYCLE = atoi(s);});
  parser.option(0, "cbpPC", 1, [&](const char* s){CBP_PC_LENGTH = atoi(s);});
  parser.option(0, "cbpBHR", 1, [&](const char* s){CBP_BHR_LENGTH = atoi(s);});
  parser.option(0, "ibpPC", 1, [&](const char* s){IBP_PC_LENGTH = atoi(s);});
  parser.option(0, "ibpBHR", 1, [&](const char* s){IBP_BHR_LENGTH = atoi(s);});
  parser.option('t', 0, 0, [&](const char* s){ENABLE_TRACE_CACHE = true;});

  parser.option(0, "fq"  , 1, [&](const char* s){FETCH_QUEUE_SIZE = atoi(s);});
  parser.option(0, "al"  , 1, [&](const char* s){ACTIVE_LIST_SIZE = atoi(s);});
  parser.option(0, "prf"  , 1, [&](const char* s){PRF_SIZE = atoi(s); AUTO_PRF_SIZE = false;});
  parser.option(0, "iq"  , 1, [&](const char* s){ISSUE_QUEUE_SIZE = atoi(s);});
  parser.option(0, "iqnp", 1, [&](const char* s){ISSUE_QUEUE_NUM_PARTS = atoi(s);});
  parser.option('a', 0, 0, [&](const char* s){PRESTEER = true;});
  parser.option('b', 0, 0, [&](const char* s){IDEAL_AGE_BASED = true;});
  parser.option(0, "lsq" , 1, [&](const char* s){LQ_SIZE = atoi(s);SQ_SIZE = atoi(s);});
  parser.option(0, "disambig", 1, [&](const char* s){set_disambig_flags(s);});
  parser.option(0, "fw"  , 1, [&](const char* s){FETCH_WIDTH = atoi(s);});
  parser.option(0, "dw"  , 1, [&](const char* s){DISPATCH_WIDTH = atoi(s);});
  parser.option(0, "iw"  , 1, [&](const char* s){ISSUE_WIDTH = atoi(s);});
  parser.option(0, "rw"  , 1, [&](const char* s){RETIRE_WIDTH = atoi(s);});
  parser.option(0, "phase",1, [&](const char *s){phase_interval = atoll(s);});
  parser.option(0, "lane" ,1, [&](const char *s){set_lane_matrix(s);});
  parser.option(0, "lat"  ,1, [&](const char *s){set_lane_latencies(s);});
  parser.option('u', 0, 0, [&](const char* s){set_lane_matrix("0xffff:0xffff:0xffff:0xffff:0xffff:0xffff:0xffff"); set_lane_latencies("1:1:1:1:1:1:1");});

  auto argv1 = parser.parse(argv);
  if (!*argv1)
    help();
  std::vector<std::string> htif_args(argv1, (const char*const*)argv + argc);

  #ifdef RISCV_MICRO_CHECKER
  s_isa = new sim_t(nprocs, mem_mb, htif_args, ISA_SIM);
  #endif

  s_micro = new sim_t(nprocs, mem_mb, htif_args, MICRO_SIM);

  s_micro->set_debug(debug);
  s_micro->set_histogram(histogram);

  #ifdef RISCV_MICRO_CHECKER
    DB = new debug_buffer_t(PIPE_QUEUE_SIZE);

    DB->set_isa_sim(s_isa);

    s_isa->set_procs_pipe(DB);
    s_micro->set_procs_pipe(DB);
  #endif

  int i, exit_code, exec_index;
  char c, *all_options;

  /* opening banner */

  /* 2/14/18 ER: Add banner for 721 simulator. */
  fprintf(stderr,
	  "\nCopyright (c) 2018 by Eric Rotenberg.  All Rights Reserved.\n");
  fprintf(stderr,
	  "Welcome to the ECE 721 Simulator.  This is a custom simulator\n");
  fprintf(stderr,
          "developed at North Carolina State University by Eric Rotenberg\n");
  fprintf(stderr,
	  "and his students.  It uses the RISCV ISA and functional simulation tools.\n\n");

  ///* print out the program arguments */
  //fprintf(stderr, "**ARGS**: ");
  //for (i=0; i<argc; i++)
  //  fprintf(stderr, "%s ", argv[i]);
  //fprintf(stderr, "\n");

  //if (argc < 2)
  //  usage(argc, argv);

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = endSimulation;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT,   &sigIntHandler, NULL);

  /* catch SIGUSR1 and dump intermediate stats */
  sigaction(SIGUSR1,  &sigIntHandler, NULL);

  /* catch SIGUSR1 and dump intermediate stats */
  sigaction(SIGUSR2,  &sigIntHandler, NULL);

  /* register an error handler */
  //fatal_hook(sim_stats);

  /* set up a non-local exit point */
  //if ((exit_code = setjmp(sim_exit_buf)) != 0)
  //{
  //  /* special handling as longjmp cannot pass 0 */
  //  exit_now(exit_code-1);
  //}

  ///* compute legal options */
  //all_options = getopt_combine_options("v", sim_optstring);
  //all_options = getopt_combine_options(all_options, mem_optstring);

  ///* parse global options */
  //getopt_init();
  //while ((c = getopt_next(argc, argv, all_options)) != EOF)
  //{
  //  switch (c) {
  //  case 'v':
	//    verbose = TRUE;
	//    break;
  //  case '?':
	//    usage(argc, argv);
  //  }
  //}
  //exec_index = getopt_index;

  ///* parse simulator options */
  //sim_options(argc, argv);
  //mem_options(argc, argv);

  //if (exec_index == argc)
  //  usage(argc, argv);

  //// 10/3/99 ER: Added support for multiple (funcsim) threads.
  //FILE *fp = fopen(argv[exec_index], "r");
  //if (fp)
  //   init_threads(fp, envp);
  //else
  //   fatal("Could not open the job file '%s'.", argv[exec_index]);

  /* record start of execution time, used in rate stats */
  start_time = time((time_t *)NULL);

  /* output simulation conditions */
  sim_config(stderr);
  mem_config(stderr);

  //pipe->skip_till_pc(0x10000,0);

  //logging_on = true;

  int htif_code;

  // Turn on logging if user requested logging from the start.
  // This way even run_ahead instructions will be logged.
  if(logging_on_at == -1)
    logging_on = true;

  #ifdef RISCV_MICRO_CHECKER
    s_isa->boot();

    if (checkpoint_file != "")
    {
      fprintf(stderr, "Restoring checkpoint from %s\n",checkpoint_file.c_str());
      s_isa->restore_checkpoint(checkpoint_file);
    }
    else if (skip_enable) {
      // If skip amount is provided, fast skip in the ISA sim
      //s_isa->init_checkpoint("isa_checkpoint");
      fprintf(stderr, "Fast skipping Spike for %lu instructions\n",skip_amt);
      htif_code = s_isa->run_fast(skip_amt);
      //htif_code = s_isa->create_checkpoint();
    }

    // Fill the debug buffer
    DB->run_ahead();
  #endif


  s_micro->boot();
  //exit(0);

  if (checkpoint_file != "")
  {
      fprintf(stderr, "Restoring checkpoint from %s\n",checkpoint_file.c_str());
      s_micro->restore_checkpoint(checkpoint_file);
  }
  else if (skip_enable) {
      // If skip amount is provided, fast skip in the MICROS sim
      fprintf(stderr, "Fast skipping MICROS for %lu instructions\n",skip_amt);
      htif_code = s_micro->run_fast(skip_amt);
      // Stop simulation if HTIF returns non-zero code
      if(!htif_code) return htif_code;
  }

  //htif_code = s_micro->create_checkpoint();
  // Stop simulation if HTIF returns non-zero code
  //if(!htif_code) return htif_code;

  // Turn on logging if user requested logging from the start of timing simulation.
  if(logging_on_at == 0)
    logging_on = true;

  fprintf(stderr, "Starting MICROS\n");
  htif_code = s_micro->run();
  fprintf(stderr, "Stopping MICROS: HTIF Exit Code %d\n",htif_code);

  //*** Must delete the simulator instances in order to dump stats ***
  // Stats are dumped in the destructor for the processor instances.
  delete s_isa;
  delete s_micro;

  return htif_code;
}
