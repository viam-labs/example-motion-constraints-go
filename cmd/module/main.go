// Module entry point for viam:example-motion-constraints-go.
//
// Registers a single rdk:service:world_state_store model that orchestrates
// scripted motion scenarios on a grid of simulated arms. See the package
// motionconstraints for the service implementation.
package main

import (
	"fmt"
	"os"
	"syscall"

	motionconstraints "motionconstraints"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
)

// mpNumThreadsDefault caps cbirrt's per-PlanMotion worker goroutine count.
// armplanning reads MP_NUM_THREADS at its package init() time, which runs
// before main() — so we can't influence it with os.Setenv from here in a
// single execution. Instead we re-exec ourselves with the env var set,
// letting the second invocation's armplanning.init() pick it up.
//
// Cap=2 keeps each plan to two CPU-bound workers. Combined with our
// max_concurrent_plans=2 semaphore, that's at most 4 worker goroutines
// running concurrently for planning — leaves plenty of scheduler headroom
// even on smaller hosts, and lets the demo show off real motion-planning
// problems (tight linear constraints, narrow corridors) without the
// planner saturating the host and starving the 3D viewer's WebRTC stream.
const mpNumThreadsDefault = "2"

func main() {
	if os.Getenv("MP_NUM_THREADS") == "" {
		env := append(os.Environ(), "MP_NUM_THREADS="+mpNumThreadsDefault)
		if err := syscall.Exec(os.Args[0], os.Args, env); err != nil {
			fmt.Fprintf(os.Stderr, "re-exec to set MP_NUM_THREADS failed: %v; continuing with default thread count\n", err)
			// Fall through — the module still works, just with more
			// cbirrt threads per plan.
		}
	}
	module.ModularMain(
		resource.APIModel{API: worldstatestore.API, Model: motionconstraints.Model},
	)
}
