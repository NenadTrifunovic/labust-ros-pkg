#ifndef SYSTEMDIAGNOSTICSNODE_HPP_
#define SYSTEMDIAGNOSTICSNODE_HPP_
#include <ros/ros.h>
#include <labust/diagnostics/BatteryDiagnostics.hpp>
#include <labust/diagnostics/ThrusterDiagnostics.hpp>

namespace labust {
  namespace diagnostics {
    /**
     * Class for system diagnostics and monitoring.
     */
    class SystemDiagnosticsNode {

      public:
        SystemDiagnosticsNode();
        ~SystemDiagnosticsNode();

      private:
        void onInit();
        BatteryDiagnostics battery_diagnostics;
        ThrusterDiagnostics thruster_diagnostics;
    };

  }
}

/* SYSTEMDIAGNOSTICSNODE_HPP_ */
#endif
