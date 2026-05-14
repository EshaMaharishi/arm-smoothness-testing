package main

import (
	"context"
	"fmt"
	"math"
	"os"
	"strconv"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

const (
	machineAddr  = "viam-lab-sanding-25-1-main.2d8xnaiovq.viam.cloud"
	apiKeyID     = "6e4b7936-902d-4ef9-9e92-ef4c80634786"
	apiKey       = "frpld1s9ozlso55pci42q3gw7upvz6r4"
	armComponent = "arm"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
}

func run() error {
	if len(os.Args) != 2 {
		return fmt.Errorf("usage: %s <N>  (number of back-and-forth cycles)", os.Args[0])
	}
	n, err := strconv.Atoi(os.Args[1])
	if err != nil || n < 1 {
		return fmt.Errorf("N must be a positive integer, got %q", os.Args[1])
	}

	ctx := context.Background()
	logger := logging.NewLogger("client")

	machine, err := client.New(
		ctx,
		machineAddr,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			apiKeyID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			},
		)),
	)
	if err != nil {
		return fmt.Errorf("failed to connect: %w", err)
	}
	defer machine.Close(ctx)

	motionSvc, err := motion.FromRobot(machine, "builtin")
	if err != nil {
		return fmt.Errorf("failed to get motion service: %w", err)
	}

	orientation := &spatialmath.OrientationVectorDegrees{
		OX:    -0.003941751303599542,
		OY:    -0.04534974253649805,
		OZ:    -0.9989633944487326,
		Theta: 168.56985540537084,
	}
	poseA := spatialmath.NewPose(
		r3.Vector{X: -599.9830423012838, Y: 757.1315452684529, Z: 245.99186541567963},
		orientation,
	)
	poseB := spatialmath.NewPose(
		r3.Vector{X: 600.0, Y: 757.1315452684529, Z: 245.99186541567963},
		orientation,
	)
	pifA := referenceframe.NewPoseInFrame(referenceframe.World, poseA)
	pifB := referenceframe.NewPoseInFrame(referenceframe.World, poseB)

	dx := poseB.Point().X - poseA.Point().X
	dy := poseB.Point().Y - poseA.Point().Y
	dz := poseB.Point().Z - poseA.Point().Z
	distMm := math.Sqrt(dx*dx + dy*dy + dz*dz)
	logger.Infof("cartesian leg distance: %.2f mm (%.2f cm)", distMm, distMm/10)

	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 2.0, OrientationToleranceDegs: 5.0},
		},
	}

	move := func(label string, dest *referenceframe.PoseInFrame) error {
		logger.Infof("moving %s in a straight line to %s %+v", armComponent, label, dest.Pose().Point())
		start := time.Now()
		_, err := motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: armComponent,
			Destination:   dest,
			Constraints:   constraints,
		})
		elapsed := time.Since(start)
		if err != nil {
			return fmt.Errorf("Move to %s failed: %w", label, err)
		}
		speedMmS := distMm / elapsed.Seconds()
		logger.Infof("  -> %s leg: %.3fs, effective speed %.2f mm/s (%.2f cm/s)",
			label, elapsed.Seconds(), speedMmS, speedMmS/10)
		return nil
	}

	for i := 1; i <= n; i++ {
		logger.Infof("cycle %d/%d", i, n)
		if err := move("A", pifA); err != nil {
			return err
		}
		if err := move("B", pifB); err != nil {
			return err
		}
	}
	logger.Infof("done: %d back-and-forth cycles complete", n)
	return nil
}
