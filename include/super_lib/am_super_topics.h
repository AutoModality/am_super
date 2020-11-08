#ifndef AM_SUPER_LIB_TOPICS_H
#define AM_SUPER_LIB_TOPICS_H

#include <string>

namespace am
{

class am_super_topics
{
public:
  /** @brief Operator interacting with the system */
	static constexpr char OPERATOR_COMMAND[] = "/operator/command";
    
  /** Controller of the mission sends State to advance through lifecycle */
	static constexpr char CONTROLLER_STATE[] = "/controller/state";

  /** State of super as reported by super */
  static constexpr char SUPER_STATE[] = "/vstate/summary";

  /** State of lifecycle nodes */
  static constexpr char LIFECYCLE_STATE[] = "/node_state";

  static constexpr char LIFECYCLE_COMMAND[] = "/node_lifecycle";

  static constexpr char SUPER_STATUS[] = "/super/status";

  static constexpr char PROCESS_STATUS[] = "process/status";
};

}

#endif
