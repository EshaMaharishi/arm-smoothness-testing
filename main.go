package main

import (
	"context"
	"encoding/csv"
	"flag"
	"fmt"
	"math"
	"os"
	"os/exec"
	"strconv"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

type armPreset struct {
	machineAddr  string
	apiKeyID     string
	apiKey       string
	armComponent string
	pointA       r3.Vector
	pointB       r3.Vector
	orientation  *spatialmath.OrientationVectorDegrees
	// allowedSelfCollisions specifies frame pairs whose geometric overlap should not be
	// treated as a self-collision by the planner. Used to work around models whose
	// adjacent-link collision meshes always overlap.
	allowedSelfCollisions [][2]string
}

var armPresets = map[string]armPreset{
	"ur20": {
		machineAddr:  "viam-lab-sanding-25-1-main.2d8xnaiovq.viam.cloud",
		apiKeyID:     "6e4b7936-902d-4ef9-9e92-ef4c80634786",
		apiKey:       "frpld1s9ozlso55pci42q3gw7upvz6r4",
		armComponent: "arm",
		pointA:       r3.Vector{X: -599.9830423012838, Y: 757.1315452684529, Z: 245.99186541567963},
		pointB:       r3.Vector{X: 600.0, Y: 757.1315452684529, Z: 245.99186541567963},
		orientation: &spatialmath.OrientationVectorDegrees{
			OX:    -0.003941751303599542,
			OY:    -0.04534974253649805,
			OZ:    -0.9989633944487326,
			Theta: 168.56985540537084,
		},
	},
	"xarm": {
		machineAddr:  "orbital-sanding-with-sanding-module-main.38o17lvxqs.viam.cloud",
		apiKeyID:     "1f680a81-1660-4cba-9d03-229eae694a55",
		apiKey:       "n9hy2znv2adocou5ff095qqv5rbhv1ex",
		armComponent: "arm",
		pointA:       r3.Vector{X: 274.89778684139594, Y: 417.6507497666596, Z: 397.2512662619799},
		pointB:       r3.Vector{X: -320.47808425325655, Y: 383.789967956841, Z: 397.25126626197994},
		orientation: &spatialmath.OrientationVectorDegrees{
			OX:    0.006662993589350791,
			OY:    0.055767008417008226,
			OZ:    -0.9984215769346364,
			Theta: 86.83649056391073,
		},
		// xarm6 JSON kinematics has perpetually-overlapping collision geometries on
		// adjacent link pairs — without these exemptions every IK solution is rejected.
		allowedSelfCollisions: [][2]string{
			{"arm:link1", "arm:link2"},
			{"arm:link2", "arm:link3"},
			{"arm:link3", "arm:link4"},
			{"arm:link4", "arm:link5"},
			{"arm:link5", "arm:link6"},
		},
	},
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
}

func run() error {
	fs := flag.NewFlagSet(os.Args[0], flag.ContinueOnError)
	armFlag := fs.String("arm", "ur20", "which arm preset to use: ur20 or xarm")
	mode := fs.String("mode", "linear", "linear (motion-service straight-line Cartesian) or waypoints (per-step IK + MoveToJointPositions)")
	spacingMm := fs.Float64("spacing-mm", 2.0, "waypoint spacing in mm (waypoints mode only)")
	n := fs.Int("n", 1, "number of back-and-forth cycles")
	if err := fs.Parse(os.Args[1:]); err != nil {
		return err
	}
	if *spacingMm <= 0 {
		return fmt.Errorf("-spacing-mm must be > 0, got %v", *spacingMm)
	}
	if *n < 1 {
		return fmt.Errorf("-n must be a positive integer, got %d", *n)
	}
	if fs.NArg() != 0 {
		return fmt.Errorf("usage: %s [-arm=ur20|xarm] [-mode=linear|waypoints] [-spacing-mm=2] [-n=1]", os.Args[0])
	}
	preset, ok := armPresets[*armFlag]
	if !ok {
		return fmt.Errorf("unknown -arm %q (want ur20 or xarm)", *armFlag)
	}
	if preset.orientation == nil {
		return fmt.Errorf("-arm %q has no pose A/B configured yet", *armFlag)
	}

	ctx := context.Background()
	logger := logging.NewLogger("client")

	logger.Infof("using arm preset %q (machine=%s, component=%s)", *armFlag, preset.machineAddr, preset.armComponent)
	machine, err := client.New(
		ctx,
		preset.machineAddr,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			preset.apiKeyID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: preset.apiKey,
			},
		)),
	)
	if err != nil {
		return fmt.Errorf("failed to connect: %w", err)
	}
	defer machine.Close(ctx)

	poseA := spatialmath.NewPose(preset.pointA, preset.orientation)
	poseB := spatialmath.NewPose(preset.pointB, preset.orientation)
	distMm := preset.pointB.Sub(preset.pointA).Norm()
	logger.Infof("cartesian leg distance: %.2f mm (%.2f cm)", distMm, distMm/10)

	switch *mode {
	case "linear":
		return runLinear(ctx, logger, machine, preset.armComponent, poseA, poseB, distMm, *n)
	case "waypoints":
		return runWaypoints(ctx, logger, machine, preset.armComponent, poseA, poseB, distMm, *n, *spacingMm, preset.allowedSelfCollisions)
	default:
		return fmt.Errorf("unknown mode %q (want linear or waypoints)", *mode)
	}
}

func runLinear(ctx context.Context, logger logging.Logger, machine *client.RobotClient, armComponent string, poseA, poseB spatialmath.Pose, distMm float64, n int) error {
	motionSvc, err := motion.FromRobot(machine, "builtin")
	if err != nil {
		return fmt.Errorf("failed to get motion service: %w", err)
	}

	pifA := referenceframe.NewPoseInFrame(referenceframe.World, poseA)
	pifB := referenceframe.NewPoseInFrame(referenceframe.World, poseB)

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

func runWaypoints(ctx context.Context, logger logging.Logger, machine *client.RobotClient, armComponent string, poseA, poseB spatialmath.Pose, distMm float64, n int, spacingMm float64, allowedSelfCollisions [][2]string) error {
	logger.Infof("waypoint spacing: %.3f mm", spacingMm)

	var planConstraints *motionplan.Constraints
	if len(allowedSelfCollisions) > 0 {
		allows := make([]motionplan.CollisionSpecificationAllowedFrameCollisions, len(allowedSelfCollisions))
		for i, pair := range allowedSelfCollisions {
			allows[i] = motionplan.CollisionSpecificationAllowedFrameCollisions{Frame1: pair[0], Frame2: pair[1]}
		}
		planConstraints = &motionplan.Constraints{
			CollisionSpecification: []motionplan.CollisionSpecification{{Allows: allows}},
		}
		logger.Infof("allowing self-collisions between %d frame pair(s)", len(allows))
	}
	a, err := arm.FromRobot(machine, armComponent)
	if err != nil {
		return fmt.Errorf("failed to get arm: %w", err)
	}

	model, err := a.Kinematics(ctx)
	if err != nil {
		return fmt.Errorf("failed to get arm kinematics: %w", err)
	}
	armName := model.Name()

	fsys := referenceframe.NewEmptyFrameSystem("client")
	if err := fsys.AddFrame(model, fsys.World()); err != nil {
		return fmt.Errorf("failed to add arm frame: %w", err)
	}

	curJoints, err := a.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("failed to read current joint positions: %w", err)
	}
	curPose, err := a.EndPosition(ctx, nil)
	if err != nil {
		return fmt.Errorf("failed to read current end position: %w", err)
	}

	// Pre-plan all joint configs we'll need.
	// Leg 1 (one-time): current pose -> poseA.
	// Leg fwd  (per cycle): poseA -> poseB.
	// Leg back (per cycle): poseB -> poseA.
	logger.Infof("planning waypoints from current -> A (warm-up leg)")
	warmupConfigs, err := planLeg(ctx, logger, fsys, armName, curPose, poseA, curJoints, spacingMm, planConstraints)
	if err != nil {
		return fmt.Errorf("warm-up plan failed: %w", err)
	}
	seedAtA := warmupConfigs[len(warmupConfigs)-1]

	logger.Infof("planning waypoints from A -> B (~%d steps)", numSteps(distMm, spacingMm))
	fwdConfigs, err := planLeg(ctx, logger, fsys, armName, poseA, poseB, seedAtA, spacingMm, planConstraints)
	if err != nil {
		return fmt.Errorf("A->B plan failed: %w", err)
	}
	seedAtB := fwdConfigs[len(fwdConfigs)-1]

	logger.Infof("planning waypoints from B -> A (~%d steps)", numSteps(distMm, spacingMm))
	revConfigs, err := planLeg(ctx, logger, fsys, armName, poseB, poseA, seedAtB, spacingMm, planConstraints)
	if err != nil {
		return fmt.Errorf("B->A plan failed: %w", err)
	}

	type moveTiming struct {
		leg   string
		idx   int
		start time.Time
		end   time.Time
	}
	var timings []moveTiming

	executeLeg := func(label string, configs [][]referenceframe.Input) error {
		logger.Infof("executing %s (%d waypoints)", label, len(configs))
		start := time.Now()
		for i, jc := range configs {
			t0 := time.Now()
			if err := a.MoveToJointPositions(ctx, jc, nil); err != nil {
				return fmt.Errorf("MoveToJointPositions failed on %s: %w", label, err)
			}
			t1 := time.Now()
			timings = append(timings, moveTiming{leg: label, idx: i, start: t0, end: t1})
		}
		elapsed := time.Since(start)
		speedMmS := distMm / elapsed.Seconds()
		logger.Infof("  -> %s: %.3fs, effective speed %.2f mm/s (%.2f cm/s)",
			label, elapsed.Seconds(), speedMmS, speedMmS/10)
		return nil
	}

	if err := executeLeg("warm-up current->A", warmupConfigs); err != nil {
		return err
	}
	for i := 1; i <= n; i++ {
		logger.Infof("cycle %d/%d", i, n)
		if err := executeLeg("A->B", fwdConfigs); err != nil {
			return err
		}
		if err := executeLeg("B->A", revConfigs); err != nil {
			return err
		}
	}
	logger.Infof("done: %d waypoint cycles complete", n)

	csvPath := "timings.csv"
	pngPath := "timings.png"
	if err := writeTimingCSV(csvPath, func(yield func(string, int, time.Time, time.Time) bool) {
		for _, t := range timings {
			if !yield(t.leg, t.idx, t.start, t.end) {
				return
			}
		}
	}); err != nil {
		logger.Warnf("failed to write timing csv: %v", err)
		return nil
	}
	logger.Infof("timing data written to %s (%d MoveToJointPositions calls)", csvPath, len(timings))

	python := ".venv/bin/python3"
	if _, err := os.Stat(python); err != nil {
		python = "python3"
	}
	if out, err := exec.Command(python, "plot_timings.py", csvPath, pngPath).CombinedOutput(); err != nil {
		logger.Warnf("plot generation failed: %v\n%s", err, string(out))
	} else {
		if len(out) > 0 {
			logger.Infof("plot: %s", string(out))
		}
		logger.Infof("plot saved to %s", pngPath)
	}
	return nil
}

func writeTimingCSV(path string, iter func(yield func(string, int, time.Time, time.Time) bool)) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	w := csv.NewWriter(f)
	defer w.Flush()
	if err := w.Write([]string{"leg", "idx_in_leg", "start_unix_s", "end_unix_s", "duration_s"}); err != nil {
		return err
	}
	var writeErr error
	iter(func(leg string, idx int, start, end time.Time) bool {
		row := []string{
			leg,
			strconv.Itoa(idx),
			strconv.FormatFloat(float64(start.UnixNano())/1e9, 'f', 9, 64),
			strconv.FormatFloat(float64(end.UnixNano())/1e9, 'f', 9, 64),
			strconv.FormatFloat(end.Sub(start).Seconds(), 'f', 6, 64),
		}
		if err := w.Write(row); err != nil {
			writeErr = err
			return false
		}
		return true
	})
	return writeErr
}

func numSteps(distMm, spacingMm float64) int {
	return int(math.Ceil(distMm/spacingMm)) + 1
}

func planLeg(
	ctx context.Context,
	logger logging.Logger,
	fsys *referenceframe.FrameSystem,
	armName string,
	from, to spatialmath.Pose,
	seedJoints []referenceframe.Input,
	spacingMm float64,
	constraints *motionplan.Constraints,
) ([][]referenceframe.Input, error) {
	fromP := from.Point()
	toP := to.Point()
	delta := toP.Sub(fromP)
	dist := delta.Norm()
	steps := int(math.Ceil(dist/spacingMm)) + 1
	if steps < 2 {
		steps = 2
	}
	// constant orientation throughout the leg (preset's orientation, which is shared by A and B)
	orient := to.Orientation()

	configs := make([][]referenceframe.Input, 0, steps)
	prevSeed := seedJoints
	for i := range steps {
		t := float64(i) / float64(steps-1)
		pt := r3.Vector{
			X: fromP.X + delta.X*t,
			Y: fromP.Y + delta.Y*t,
			Z: fromP.Z + delta.Z*t,
		}
		waypoint := spatialmath.NewPose(pt, orient)
		pif := referenceframe.NewPoseInFrame(referenceframe.World, waypoint)
		goalState := armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{armName: pif}, nil)
		startState := armplanning.NewPlanState(
			nil, referenceframe.FrameSystemInputs{armName: prevSeed})
		req := &armplanning.PlanRequest{
			FrameSystem: fsys,
			Goals:       []*armplanning.PlanState{goalState},
			StartState:  startState,
			Constraints: constraints,
		}
		plan, _, err := armplanning.PlanMotion(ctx, logger, req)
		if err != nil {
			return nil, fmt.Errorf("PlanMotion failed at waypoint %d/%d (pt=%v): %w", i+1, steps, pt, err)
		}
		traj := plan.Trajectory()
		endInputs := traj[len(traj)-1][armName]
		configs = append(configs, endInputs)
		prevSeed = endInputs
	}
	logger.Infof("planned %d joint configs over %.2f mm", len(configs), dist)
	return configs, nil
}
