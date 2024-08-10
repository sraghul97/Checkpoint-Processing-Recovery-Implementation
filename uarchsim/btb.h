
// A BTB entry.
typedef
struct {
   // Metadata for hit/miss determination and replacement.
   bool valid;
   uint64_t tag;
   uint64_t lru;

   // Payload.
   btb_branch_type_e branch_type;
   uint64_t target;
} btb_entry_t;



class btb_t {
private:
	// The BTB has three dimensions: number of banks, number of sets per bank, and associativity (number of ways per set).
	// btb[bank][set][way]
	btb_entry_t ***btb;
	uint64_t banks;
	uint64_t sets;
	uint64_t assoc;

	uint64_t log2banks; // number of pc bits that selects the bank
	uint64_t log2sets;  // number of pc bits that selects the set within a bank

	uint64_t cond_branch_per_cycle; // "m": maximum number of conditional branches in a fetch bundle.

	////////////////////////////////////
	// Private utility functions.
	// Comments are in btb.cc.
	////////////////////////////////////

	void convert(uint64_t pc, uint64_t pos, uint64_t &btb_bank, uint64_t &btb_pc);
	bool search(uint64_t btb_bank, uint64_t btb_pc, uint64_t &set, uint64_t &way);
	void update_lru(uint64_t btb_bank, uint64_t set, uint64_t way);
	

public:
	btb_t(uint64_t num_entries, uint64_t banks, uint64_t assoc, uint64_t cond_branch_per_cycle);
	~btb_t();
        void lookup(uint64_t pc, uint64_t cb_predictions, uint64_t ib_predicted_target, uint64_t ras_predicted_target, fetch_bundle_t bundle[], spec_update_t *update);
	void update(uint64_t pc, uint64_t pos, insn_t insn);
	void invalidate(uint64_t pc, uint64_t pos);
	static btb_branch_type_e decode(insn_t insn, uint64_t pc, uint64_t &target);
};
