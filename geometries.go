package motionconstraints

import (
	"encoding/base64"
	"math"

	commonpb "go.viam.com/api/common/v1"
	"google.golang.org/protobuf/types/known/structpb"
)

// Color is the RGB color used in scene metadata. Channels are 0..255.
type Color struct {
	R, G, B int
}

var (
	// ColorObstacle is the default tint for static obstacles.
	ColorObstacle = Color{R: 80, G: 80, B: 200}
	// ColorTrajectory is the tint for ghost-trajectory waypoints.
	ColorTrajectory = Color{R: 0, G: 200, B: 120}
	// ColorCollision is the red used to highlight collision participants.
	ColorCollision = Color{R: 255, G: 0, B: 0}
	// ColorGoal is the tint for goal markers.
	ColorGoal = Color{R: 230, G: 180, B: 0}
)

// metadataOpts collects the inputs to buildMetadata. Every field except
// Color is independently optional; the renderer requires all five output
// keys to be present so we fill in defaults below.
type metadataOpts struct {
	Color          *Color
	Opacity        *float64
	ShowAxesHelper bool
	Invisible      bool
}

// buildMetadata encodes the metadata struct the 3D viewer reads. All five
// required keys (colors, color_format, opacities, show_axes_helper,
// invisible) are always populated — omitting any of them produces an
// invisible entity. See CLAUDE.md "renderer conventions".
func buildMetadata(opts metadataOpts) *structpb.Struct {
	fields := map[string]any{}

	if opts.Color != nil {
		rgb := []byte{
			byte(clampU8(opts.Color.R)),
			byte(clampU8(opts.Color.G)),
			byte(clampU8(opts.Color.B)),
		}
		fields["colors"] = base64.StdEncoding.EncodeToString(rgb)
	} else {
		fields["colors"] = ""
	}
	fields["color_format"] = 1.0 // COLOR_FORMAT_RGB

	alpha := 255
	if opts.Opacity != nil {
		alpha = clampU8(int(math.Round(*opts.Opacity * 255)))
	}
	fields["opacities"] = base64.StdEncoding.EncodeToString([]byte{byte(alpha)})
	fields["show_axes_helper"] = opts.ShowAxesHelper
	fields["invisible"] = opts.Invisible

	s, err := structpb.NewStruct(fields)
	if err != nil {
		return nil
	}
	return s
}

func clampU8(v int) int {
	if v < 0 {
		return 0
	}
	if v > 255 {
		return 255
	}
	return v
}

func opacityPtr(v float64) *float64 { return &v }

// posePB builds a commonpb.Pose with identity orientation when no vector
// components are supplied. Position is in millimeters.
func posePB(x, y, z float64) *commonpb.Pose {
	return &commonpb.Pose{X: x, Y: y, Z: z, OZ: 1.0}
}

// boxGeometry builds a rectangular-prism geometry proto. Dimensions are in
// millimeters; label is the human-readable identifier shown in the viewer.
func boxGeometry(dx, dy, dz float64, label string) *commonpb.Geometry {
	return &commonpb.Geometry{
		Label: label,
		GeometryType: &commonpb.Geometry_Box{
			Box: &commonpb.RectangularPrism{
				DimsMm: &commonpb.Vector3{X: dx, Y: dy, Z: dz},
			},
		},
	}
}

// sphereGeometry builds a sphere geometry proto. Radius is in millimeters.
func sphereGeometry(radiusMM float64, label string) *commonpb.Geometry {
	return &commonpb.Geometry{
		Label: label,
		GeometryType: &commonpb.Geometry_Sphere{
			Sphere: &commonpb.Sphere{RadiusMm: radiusMM},
		},
	}
}
