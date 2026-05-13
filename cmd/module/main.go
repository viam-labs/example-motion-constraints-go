// Module entry point for viam:example-motion-constraints-go.
//
// Registers a single rdk:service:world_state_store model that orchestrates
// scripted motion scenarios on a grid of simulated arms. See the package
// motionconstraints for the service implementation.
package main

import (
	motionconstraints "motionconstraints"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: worldstatestore.API, Model: motionconstraints.Model},
	)
}
