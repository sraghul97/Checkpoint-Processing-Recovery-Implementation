#include "debug.h"
#include "pipeline.h"
#include "payload.h"

#include <new> // make sure we can use the placement new syntax

trap_t *trap_storage_t::get() {
	return reinterpret_cast<trap_t *>(&this->trap_storage);
}

void trap_storage_t::post(const trap_t *t) {
   if (content_valid) {
      // The trap posted first takes precedence.
      return;
   }

	if (dynamic_cast<const mem_trap_t *>(t)) {
		if (dynamic_cast<const trap_instruction_address_misaligned *>(t)) {
			new(reinterpret_cast<trap_instruction_address_misaligned *>(&trap_storage))
				trap_instruction_address_misaligned(*dynamic_cast<const trap_instruction_address_misaligned *>(t));
		} else if (dynamic_cast<const trap_instruction_access_fault *>(t)) {
			new(reinterpret_cast<trap_instruction_access_fault *>(&trap_storage))
				trap_instruction_access_fault(*dynamic_cast<const trap_instruction_access_fault *>(t));
		} else if (dynamic_cast<const trap_load_address_misaligned *>(t)) {
			new(reinterpret_cast<trap_load_address_misaligned *>(&trap_storage))
				trap_load_address_misaligned(*dynamic_cast<const trap_load_address_misaligned *>(t));
		} else if (dynamic_cast<const trap_store_address_misaligned *>(t)) {
			new(reinterpret_cast<trap_store_address_misaligned *>(&trap_storage))
				trap_store_address_misaligned(*dynamic_cast<const trap_store_address_misaligned *>(t));
		} else if (dynamic_cast<const trap_load_access_fault *>(t)) {
			new(reinterpret_cast<trap_load_access_fault *>(&trap_storage))
				trap_load_access_fault(*dynamic_cast<const trap_load_access_fault *>(t));
		} else if (dynamic_cast<const trap_store_access_fault *>(t)) {
			new(reinterpret_cast<trap_store_access_fault *>(&trap_storage))
				trap_store_access_fault(*dynamic_cast<const trap_store_access_fault *>(t));
		} else {
			assert(0);
		}
	} else {
		if (dynamic_cast<const trap_illegal_instruction *>(t)) {
			new(reinterpret_cast<trap_illegal_instruction *>(&trap_storage))
				trap_illegal_instruction(*dynamic_cast<const trap_illegal_instruction *>(t));
		} else if (dynamic_cast<const trap_privileged_instruction *>(t)) {
			new(reinterpret_cast<trap_privileged_instruction *>(&trap_storage))
				trap_privileged_instruction(*dynamic_cast<const trap_privileged_instruction *>(t));
		} else if (dynamic_cast<const trap_fp_disabled *>(t)) {
			new(reinterpret_cast<trap_fp_disabled *>(&trap_storage))
				trap_fp_disabled(*dynamic_cast<const trap_fp_disabled *>(t));
		} else if (dynamic_cast<const trap_syscall *>(t)) {
			new(reinterpret_cast<trap_syscall *>(&trap_storage))
				trap_syscall(*dynamic_cast<const trap_syscall *>(t));
		} else if (dynamic_cast<const trap_breakpoint *>(t)) {
			new(reinterpret_cast<trap_breakpoint *>(&trap_storage))
				trap_breakpoint(*dynamic_cast<const trap_breakpoint *>(t));
		} else if (dynamic_cast<const trap_accelerator_disabled *>(t)) {
			new(reinterpret_cast<trap_accelerator_disabled *>(&trap_storage))
				trap_accelerator_disabled(*dynamic_cast<const trap_accelerator_disabled *>(t));
		} else if (dynamic_cast<const trap_csr_instruction *>(t)) {
			new(reinterpret_cast<trap_csr_instruction *>(&trap_storage))
				trap_csr_instruction(*dynamic_cast<const trap_csr_instruction *>(t));
		} else {
			assert(0);
		};
	}

	content_valid = true;
}

void trap_storage_t::post(const trap_t &t) {
	post(&t);
}

void trap_storage_t::clear() {
	if (!content_valid)
		return;
	auto* t = reinterpret_cast<trap_t *>(&this->trap_storage);
	t->~trap_t();
	content_valid = false;
}

payload::payload(unsigned int total_inflight_instr) {
	assert(total_inflight_instr > 0);
	unsigned int temp = 2*total_inflight_instr; // Need two PAY entries for each in-flight instruction to support splitting.
	if (!IsPow2(temp))
	   temp = 2*temp;
	PAYLOAD_BUFFER_SIZE = 1;
	while (temp > 1) {
	   PAYLOAD_BUFFER_SIZE = (PAYLOAD_BUFFER_SIZE << 1);
	   temp = (temp >> 1);
	}
	assert(IsPow2(PAYLOAD_BUFFER_SIZE));
	assert(PAYLOAD_BUFFER_SIZE >= 2*total_inflight_instr);
	if (IsPow2(total_inflight_instr))
	   assert(PAYLOAD_BUFFER_SIZE == 2*total_inflight_instr);
	else
	   assert((PAYLOAD_BUFFER_SIZE > 2*total_inflight_instr) && (PAYLOAD_BUFFER_SIZE < 4*total_inflight_instr));

	buf = new payload_t[PAYLOAD_BUFFER_SIZE];
	clear();
}

unsigned int payload::push() {
	unsigned int index;

	index = tail;

	// Increment tail by two, since each instruction is pre-allocated
	// two entries, even and odd, to accommodate instruction splitting.
	tail = MOD((tail + 2), PAYLOAD_BUFFER_SIZE);
	length += 2;

	// Check for overflowing buf.
	assert(length <= PAYLOAD_BUFFER_SIZE);

	return(index);
}

void payload::pop() {
	// Increment head by one.
	head = MOD((head + 1), PAYLOAD_BUFFER_SIZE);
	length -= 1;

	// Check for underflowing instr_buf.
	assert(length >= 0);
}

void payload::clear() {
	head = 0;
	tail = 0;
	length = 0;
}

void payload::split(unsigned int index) {
  //TODO: Remove this assert if necessary
  assert(0); // Should not come here now
	assert((index+1) < PAYLOAD_BUFFER_SIZE);

	buf[index+1].inst             = buf[index].inst;
	buf[index+1].pc               = buf[index].pc;
	buf[index+1].next_pc          = buf[index].next_pc;
	buf[index+1].pred_tag         = buf[index].pred_tag;
	buf[index+1].good_instruction = buf[index].good_instruction;
	buf[index+1].db_index	        = buf[index].db_index;

	buf[index+1].flags            = buf[index].flags;
	buf[index+1].fu               = buf[index].fu;
	buf[index+1].latency          = buf[index].latency;
	buf[index+1].checkpoint       = buf[index].checkpoint;
	buf[index+1].split_store      = buf[index].split_store;

	buf[index+1].A_valid          = false;
	buf[index+1].B_valid          = false;
	buf[index+1].C_valid          = false;

	////////////////////////

	buf[index].split = true;
	buf[index].upper = true;

	buf[index+1].split = true;
	buf[index+1].upper = false;
}

// Mapping of instructions to actual (functional simulation) instructions.
void payload::map_to_actual(pipeline_t* proc, unsigned int index) {
	unsigned int prev_index;
	bool         first;
	debug_index_t db_index;

	//////////////////////////////
	// Find previous instruction.
	//////////////////////////////
	prev_index = MOD((index + PAYLOAD_BUFFER_SIZE - 2), PAYLOAD_BUFFER_SIZE);
	first = (index == head);

	////////////////////////////
	// Calculate and set state.
	////////////////////////////
	if (first) {                           // FIRST INSTRUCTION
		buf[index].good_instruction = true;
    //TODO: Fix this
		buf[index].db_index = proc->get_pipe()->first(buf[index].pc);
	}
	else if (buf[prev_index].good_instruction) {         // GOOD MODE
    //TODO: Fix this
		db_index = proc->get_pipe()->check_next(buf[prev_index].db_index, buf[index].pc);
		if (db_index == DEBUG_INDEX_INVALID) {
			// Transition to bad mode.
			buf[index].good_instruction = false;
			buf[index].db_index = DEBUG_INDEX_INVALID;
		}
		else {
			// Stay in good mode.
			buf[index].good_instruction = true;
			buf[index].db_index = db_index;
		}
	}
	else {                                               // BAD MODE
		// Stay in bad mode.
		buf[index].good_instruction = false;
		buf[index].db_index = DEBUG_INDEX_INVALID;
	}
}

// Perfect branch prediction, up to max_length instructions or the first indirect branch.
void payload::predict(pipeline_t *proc, uint64_t pc, uint64_t max_length, uint64_t &cb_predictions, uint64_t &indirect_target) {
   uint64_t prev;
   debug_index_t db_index;
   db_t *actual;
   uint64_t i, j;
   bool stop_indirect;

   // Get the debug buffer index of the first instruction in the fetch bundle.
   if (tail == head) {
      // There isn't a previous instruction (PAY is empty).
      db_index = proc->get_pipe()->first(pc);
   }
   else {
      // There is a previous instruction in PAY: use its debug buffer index to get that of the first instruction in the fetch bundle.
      prev = MOD((tail + PAYLOAD_BUFFER_SIZE - 2), PAYLOAD_BUFFER_SIZE);
      db_index = (buf[prev].good_instruction ? proc->get_pipe()->check_next(buf[prev].db_index, pc) : DEBUG_INDEX_INVALID);
   }

   // Initialize conditional branch predictions to all 0s.
   cb_predictions = 0;

   // i is instruction slot in fetch bundle
   // j is branch slot in cb_predictions
   i = 0;
   j = 0;
   stop_indirect = false;
   while ((i < max_length) && !stop_indirect && (db_index != DEBUG_INDEX_INVALID)) {
      actual = proc->get_pipe()->peek(db_index);
      switch (actual->a_inst.opcode()) {
	 case OP_BRANCH:
	    if (actual->a_next_pc != INCREMENT_PC(actual->a_pc))     // taken
	       cb_predictions = (cb_predictions | (3 << (j << 1)));
	    j++;
	    break;

	 case OP_JALR:
	    indirect_target = actual->a_next_pc;
	    stop_indirect = true;
	    break;
      
         default:
	    break;
      }

      i++;
      db_index = proc->get_pipe()->check_next(db_index, actual->a_next_pc);
   }
}

void payload::rollback(unsigned int index) {
	// Rollback the tail to the instruction after the instruction at 'index'.
	tail = MOD((index + 2), PAYLOAD_BUFFER_SIZE);

	// Recompute the length.
	length = MOD((PAYLOAD_BUFFER_SIZE + tail - head), PAYLOAD_BUFFER_SIZE);
}

unsigned int payload::checkpoint() {
	return(tail);
}

void payload::restore(unsigned int index) {
	// Rollback the tail to 'index'.
	tail = index;

	// Recompute the length.
	length = MOD((PAYLOAD_BUFFER_SIZE + tail - head), PAYLOAD_BUFFER_SIZE);
}

void payload::dump(pipeline_t* proc,unsigned int index,FILE* file)
{
  proc->disasm(buf[index].inst,proc->cycle,buf[index].pc,buf[index].sequence,file);
  ifprintf(logging_on,file,"next_pc    : %" PRIxreg "\t",  buf[index].next_pc);
  ifprintf(logging_on,file,"c_next_pc  : %" PRIxreg "\t",  buf[index].c_next_pc);
  ifprintf(logging_on,file,"Mem addr   : %" PRIxreg "\t",  buf[index].addr);
  ifprintf(logging_on,file,"pred_tag   : %u\t",            buf[index].pred_tag);
  ifprintf(logging_on,file,"good_inst  : %u\t",            buf[index].good_instruction);
  ifprintf(logging_on,file,"\n");
  ifprintf(logging_on,file,"pay_index  : %u\t",            index);
  ifprintf(logging_on,file,"db_index   : %u\t",            buf[index].db_index);
  ifprintf(logging_on,file,"iq         : %u\t",            buf[index].iq);
  ifprintf(logging_on,file,"\n");
  ifprintf(logging_on,file,"RS1 Valid  : %u\t",            buf[index].A_valid);
  ifprintf(logging_on,file,"RS1 Logical: %u\t",            buf[index].A_log_reg);
  ifprintf(logging_on,file,"RS1 Phys   : %u\t",            buf[index].A_phys_reg);
  ifprintf(logging_on,file,"RS1 Value  : 0x%" PRIxreg "\t",buf[index].A_value.dw);
  ifprintf(logging_on,file,"RS1 Value  : %" PRIsreg "\t",  buf[index].A_value.sdw);
  ifprintf(logging_on,file,"RS1 Value  : %f\t",            buf[index].A_value.d);
  ifprintf(logging_on,file,"\n");
  ifprintf(logging_on,file,"RS2 Valid  : %u\t",            buf[index].B_valid);
  ifprintf(logging_on,file,"RS2 Logical: %u\t",            buf[index].B_log_reg);
  ifprintf(logging_on,file,"RS2 Phys   : %u\t",            buf[index].B_phys_reg);
  ifprintf(logging_on,file,"RS2 Value  : 0x%" PRIxreg "\t",buf[index].B_value.dw);
  ifprintf(logging_on,file,"RS2 Value  : %" PRIsreg "\t",  buf[index].B_value.sdw);
  ifprintf(logging_on,file,"RS2 Value  : %f\t",            buf[index].B_value.d);
  ifprintf(logging_on,file,"\n");
  ifprintf(logging_on,file,"U   Imm    : 0x%" PRIxreg "\t",buf[index].inst.u_imm());
  ifprintf(logging_on,file,"\n");
  if(unlikely(buf[index].D_valid)){
    ifprintf(logging_on,file,"RS3 Valid  : %u\t",            buf[index].D_valid);
    ifprintf(logging_on,file,"RS3 Logical: %u\t",            buf[index].D_log_reg);
    ifprintf(logging_on,file,"RS3 Phys   : %u\t",            buf[index].D_phys_reg);
    ifprintf(logging_on,file,"RS3 Value  : 0x%" PRIxreg "\t",buf[index].D_value.dw);
    ifprintf(logging_on,file,"RS3 Value  : %" PRIsreg "\t",  buf[index].D_value.sdw);
    ifprintf(logging_on,file,"RS3 Value  : %f\t",            buf[index].D_value.d);
    ifprintf(logging_on,file,"\n");
  }
  ifprintf(logging_on,file,"RD  Valid  : %u\t",            buf[index].C_valid);
  ifprintf(logging_on,file,"RD  Logical: %u\t",            buf[index].C_log_reg);
  ifprintf(logging_on,file,"RD  Phys   : %u\t",            buf[index].C_phys_reg);
  ifprintf(logging_on,file,"RD  Value  : 0x%" PRIxreg "\t",buf[index].C_value.dw);
  ifprintf(logging_on,file,"RD  Value  : %" PRIsreg "\t",  buf[index].C_value.sdw);
  ifprintf(logging_on,file,"RD  Value  : %f\t",            buf[index].C_value.d);
  ifprintf(logging_on,file,"\n");
  ifprintf(logging_on,file,"\n");
}

unsigned int payload::get_size() {
   return(PAYLOAD_BUFFER_SIZE);
}
