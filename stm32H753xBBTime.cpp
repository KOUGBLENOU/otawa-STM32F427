/*
 *	STM32 module implementation
 *
 *	This file is part of OTAWA
 *	Copyright (c) 2017, IRIT UPS.
 *
 *	OTAWA is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	OTAWA is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with OTAWA; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <otawa/events/StandardEventBuilder.h>
#include <otawa/etime/EdgeTimeBuilder.h>
#include <otawa/prop/DynIdentifier.h>
#include <elm/sys/Path.h>
#include <otawa/loader/arm.h>
#include <elm/io/FileOutput.h>
#include <elm/data/Vector.h>
#include "M7FCycleTiming.h"
#include "arm_operand.h"

namespace otawa { namespace stm32 {
    using namespace elm::io;
    extern p::id<bool> WRITE_LOG;
    typedef enum {
        FE    = 0,
        DE    = 1,
        DPU   = 2,
        WR    = 3,
        CNT   = 4
    } pipeline_stage_t;
    
    class M7ExeGraph: public etime::EdgeTimeGraph {
	public:
		
		M7ExeGraph(WorkSpace* ws,
                 ParExeProc* proc, 
                 Vector<Resource* >* hw_resources, 
				 ParExeSequence* seq,
                 const PropList& props,
                 FileOutput* out, 
                 elm::Vector<Address>* unknown_inst_address) : etime::EdgeTimeGraph(ws, proc, hw_resources, seq, props),
				 												_unknown_inst_address(unknown_inst_address), _out(out)  {
			
			// Try to find arm loader with arm information
			DynIdentifier<arm::Info* > id("otawa::arm::Info::ID");
			info = id(_ws->process());
			if (!info)
				throw Exception("ARM loader with otawa::arm::INFO is required !");
			// Get memory configuration
			mem = hard::MEMORY_FEATURE.get(ws);
			ASSERTP(mem, "Memory feature not found");
		}

        /*
			Write to the log file, some info about the instructions whose
			cycle timing info has not been found.
		*/
		void dumpUnknowInst() {
			if (_out == nullptr)
				return;
			for (InstIterator inst(this); inst(); inst++) {
				if (!getInstCycleTiming(inst->inst())->unknown)
					continue;
				
				auto addr = inst->inst()->address();
				if (_unknown_inst_address->contains(addr))
					continue;
				_unknown_inst_address->add(addr);
				*_out << addr << "; " << inst->inst() << endl;
			}
		}
		
		void addEdgeForMulDependencies() {
			static string mul_order = "mul order";
			auto stage = _microprocessor->execStage();
			
			// looking in turn each FU
			for (int i=0 ; i<stage->numFus() ; i++) {
				ParExeStage *fu_stage = stage->fu(i)->firstStage();
				ParExeNode * previous_mul = nullptr;
				
				// look for each node of this FU
				for (int j=0 ; j<fu_stage->numNodes() ; j++) {
					ParExeNode *node = fu_stage->node(j);

					// found a mul instruction
					if (node->inst()->inst()->isMul()) {
						if (previous_mul) {
							if (previous_mul->inst()->inst() != node->inst()->inst())
								new ParExeEdge(previous_mul, node, ParExeEdge::SOLID, 0, mul_order);
						}
						// current node becomes the new previous mul
						for (InstNodeIterator last_node(node->inst()); last_node() ; last_node++)
							if (last_node->stage()->category() == ParExeStage::FU)
								previous_mul = *last_node;
					}
				}
			}
		}
		void addEdgesForFetch() override {
			ParExeGraph::addEdgesForFetch();

			ParExeInst* prev_inst = 0;
			for (InstIterator inst(this); inst(); inst++) {
				// Update fetch edges with misprediction branch penalties
				if (prev_inst && prev_inst->inst()->isControl())
					if (prev_inst->inst()->topAddress() != inst->inst()->address())
						new ParExeEdge(prev_inst->lastFUNode(), inst->fetchNode(), ParExeEdge::SOLID, 1, "Branch prediction");
				prev_inst = *inst;
			}
		}
        void addEdgesForPipelineOrder() override {
            ParExeGraph::addEdgesForPipelineOrder();
			// Add latency penalty to Exec-FU nodes
            for (InstIterator inst(this); inst(); inst++) {
				// get cycle_time_info of inst
                m7f_time_t* inst_cycle_timing = getInstCycleTiming(inst->inst());
                ot::time inst_cost = inst_cycle_timing->ex_cost;
                if (inst_cycle_timing->multi)
					inst_cost += getInstNReg(inst->inst());
                if (inst_cost > 1)
                    inst->firstFUNode()->setLatency(inst_cost - 1);

            }
        }

		
		void addEdgesForDataDependencies() override {
			string data_dep("");
			ParExeStage* exec_stage = _microprocessor->execStage();
			// for each functional unit
			for (int j = 0; j < exec_stage->numFus(); j++) {
				ParExeStage* fu_stage = exec_stage->fu(j)->firstStage();
				
				// for each stage in the functional unit
				for (int k=0; k < fu_stage->numNodes(); k++) {
					ParExeNode* node = fu_stage->node(k);
					ParExeInst* inst = node->inst();
					
					// for each instruction producing a used data
					for (ParExeInst::ProducingInstIterator prod(inst); prod(); prod ++) {
						// get cycle_time_info of the producer instruction
						m7f_time_t* producer_cycle_timing = getInstCycleTiming(prod->inst());
						
						// Calculate the stall duration of the stage
						int stall_duration = producer_cycle_timing->result_latency - producer_cycle_timing->ex_cost;
						
						// Find the stage node producing the data
						ParExeNode* producing_node = nullptr;
						if (!prod->inst()->isLoad())
							producing_node = prod->lastFUNode();
						else
							producing_node = findMemStage(*prod);
						
						// In the case of negative value of stall duration, we need to reduce the latency value of the producer node
						if (stall_duration < 0) {
							// producing_node->setLatency(producing_node->latency() - elm::abs(stall_duration));
							stall_duration = 0;
						}
						// create the edge
						if (producing_node != nullptr && node != nullptr) 
							new ParExeEdge(producing_node, node, ParExeEdge::SOLID, stall_duration, data_dep);
					}
				}
			}
		}
        
        void build() override {

            for (ParExePipeline::StageIterator pipeline_stage(_microprocessor->pipeline()); pipeline_stage(); pipeline_stage++) {
				if (pipeline_stage->name() == "PFU") {
					stage[FE] = *pipeline_stage;
				} else if (pipeline_stage->name() == "Decode") {
					stage[DE] = *pipeline_stage;
				} else if (pipeline_stage->name() == "DPU") {
					stage[DPU] = *pipeline_stage;
					for (int i = 0; i < pipeline_stage->numFus(); i++) {
						ParExePipeline* fu = pipeline_stage->fu(i);
						if (fu->firstStage()->name().startsWith("FPU")) {
							fpu_exec = fu;
						} else if (fu->firstStage()->name().startsWith("ALU")) {
							alu_exec = fu;
						} else if (fu->firstStage()->name().startsWith("BRANCH")) {
							branch_exec = fu;
						} else if (fu->firstStage()->name().startsWith("LSU")) {
							lsu_exec = fu;
						} else
							ASSERTP(false, fu->firstStage()->name());
					}
				} else if (pipeline_stage->name() == "Write") {
					stage[WR] = *pipeline_stage;
				} 

			}
			ASSERTP(stage[FE], "No 'Prefetch' stage found");
			ASSERTP(stage[DE], "No 'Decode' stage found");
			ASSERTP(stage[DPU], "No 'DPU' stage found");
			ASSERTP(stage[WR], "No 'Write back' stage found");
			ASSERTP(fpu_exec, "No FPU fu found");
			ASSERTP(alu_exec, "No ALU fu found");
			ASSERTP(lsu_exec, "No LSU fu found");
			ASSERTP(branch_exec, "No BRANCH fu found");
			
            // Build the execution graph 
			createSequenceResources();
			createNodes();
			addEdgesForPipelineOrder();
			addEdgesForFetch();
			addEdgesForProgramOrder();
			addEdgesForMemoryOrder();
			addEdgesForDataDependencies();
			dumpUnknowInst();
			addEdgeForMulDependencies();
			
        }

        private:
            otawa::arm::Info* info;
            const hard::Memory* mem;
            ParExeStage* stage[CNT];
            ParExePipeline *fpu_exec, *alu_exec, *lsu_exec, *branch_exec;
            FileOutput* _out = nullptr;
            elm::Vector<Address>* _unknown_inst_address = nullptr;


            /*
			Attempts to decode an instruction and return the corresponding behavior "cycle timing behavior".
			
			    inst: Instruction decode.
            */
            m7f_time_t* getInstCycleTiming(Inst* inst) {
                void* inst_info = info->decode(inst);
                m7f_time_t* inst_cycle_timing = stm32M7F(inst_info);
                info->free(inst_info);
                return inst_cycle_timing;
            }
            
            t::uint32 getInstNReg(Inst* inst) {
                void* inst_info = info->decode(inst);
                t::uint32 inst_n_reg = armV7_NReg(inst_info);
                info->free(inst_info);
                return inst_n_reg;
		    }
			
			/*
			Find the Mem stage of an instruction.
			    inst: Concerned instruction.
			*/
			ParExeNode* findMemStage(ParExeInst* inst) {
				for (ParExeInst::NodeIterator node(inst); node(); node++) {
						if (node->stage() == _microprocessor->memStage())
							return *node;
				}
				return nullptr;
			}

    };

    class BBTimerSTM32M7F: public etime::EdgeTimeBuilder {
	public:
		static p::declare reg;
		BBTimerSTM32M7F(void): etime::EdgeTimeBuilder(reg) { }

	protected:
		virtual void configure(const PropList& props) {
			etime::EdgeTimeBuilder::configure(props);
			write_log = WRITE_LOG(props);
			_props = props;
		}
		void setup(WorkSpace* ws) override {
			etime::EdgeTimeBuilder::setup(ws);
			const hard::CacheConfiguration* cache_config = hard::CACHE_CONFIGURATION_FEATURE.get(ws);
			if (!cache_config)
				throw ProcessorException(*this, "Cache configuration non available");

			if (write_log) {
				sys::Path log_file_path = sys::Path(ws->process()->program_name() + ".log");
				bool write_header = (log_file_path.exists()) ? false : true;
				log_stream = new FileOutput(log_file_path, true);
				if (write_header)
					*log_stream << "########################################################" << endl
								<< "# Static analysis on " << ws->process()->program_name() << endl
								<< "# Overestimated instructions" << endl
								<< "# Address (hex); Instruction" << endl
								<< "########################################################" << endl;
				else
					*log_stream << endl; // sep
				
				unknown_inst_address = new elm::Vector<Address>();
			}
		}

		etime::EdgeTimeGraph* make(ParExeSequence* seq) override {
			M7ExeGraph* graph = new M7ExeGraph(workspace(), _microprocessor, ressources(), seq, _props, log_stream, unknown_inst_address);
			graph->build();
			return graph;
		}


		virtual void clean(ParExeGraph* graph) {
			log_stream->flush();
			delete graph;
		}
	private:
		PropList _props;
		FileOutput* log_stream = nullptr;
		bool write_log = 0;
		elm::Vector<Address>* unknown_inst_address = nullptr;
	};

	

	p::declare BBTimerSTM32M7F::reg = p::init("otawa::stm32::BBTimerSTM32M7F", Version(1, 0, 0))
										.extend<etime::EdgeTimeBuilder>()
										.maker<BBTimerSTM32M7F>();
	
} // namespace stm32
} // namespace otawa
