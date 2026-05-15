package motionconstraints

import (
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"sync"
)

// ModuleDir is the directory where the module was extracted by
// viam-server — the tarball root. Assets live at <ModuleDir>/assets/.
//
// Resolved from os.Executable() at first use: the binary is at
// <ModuleDir>/bin/example-motion-constraints-go, so the module dir is
// two parents up. CWD when viam-server launches the module isn't
// reliable, so the executable path is the only stable anchor.
var (
	moduleDirOnce sync.Once
	moduleDir     string
)

func resolveModuleDir() string {
	moduleDirOnce.Do(func() {
		exe, err := os.Executable()
		if err == nil {
			candidate := filepath.Dir(filepath.Dir(exe))
			if dirHas(candidate, "assets") {
				moduleDir = candidate
				return
			}
		}
		// Dev fallback: CWD when running `go run` or tests.
		if cwd, err := os.Getwd(); err == nil && dirHas(cwd, "assets") {
			moduleDir = cwd
		}
	})
	return moduleDir
}

func dirHas(dir, name string) bool {
	info, err := os.Stat(filepath.Join(dir, name))
	return err == nil && info.IsDir()
}

// labelAssetFilenameSafeRe matches characters disallowed in label asset
// filenames. Mirrors scripts/generate_text_assets.py's safe-name regex.
var labelAssetFilenameSafeRe = regexp.MustCompile(`[^A-Za-z0-9_-]`)

// labelAssetFilename returns the on-disk PLY filename for a label
// string at the given height. Must match the Python generator's
// label_asset_filename so the Go loader finds the file the script
// wrote.
func labelAssetFilename(text string, heightMM int) string {
	safe := labelAssetFilenameSafeRe.ReplaceAllString(text, "_")
	return fmt.Sprintf("text__%dmm__%s.ply", heightMM, safe)
}

// loadTextPLY reads the PLY bytes for a pre-generated text mesh asset.
// Returns a friendly error if the asset is missing — typically means
// the user added a new label string without re-running `make assets`
// (or the underlying Python generator script).
func loadTextPLY(text string, heightMM int) ([]byte, error) {
	dir := resolveModuleDir()
	if dir == "" {
		return nil, fmt.Errorf("could not resolve module dir for text-PLY lookup (expected an <ModuleDir>/assets/ next to the binary)")
	}
	name := labelAssetFilename(text, heightMM)
	full := filepath.Join(dir, "assets", name)
	data, err := os.ReadFile(full)
	if err != nil {
		return nil, fmt.Errorf("loadTextPLY %q: %w (regenerate via `make assets` after editing LABELS)", text, err)
	}
	return data, nil
}

// labelTextForArm returns the multi-line description of what an arm
// is demonstrating, based on its scenario key and whether it has an
// offset gripper attached. Used as the source string for the
// pre-generated text PLY asset. Must match a LABELS entry in
// scripts/generate_text_assets.py — re-run `make assets` after edits.
func labelTextForArm(scenarioKey string, hasGripper bool) string {
	switch scenarioKey {
	case "random_translation":
		if hasGripper {
			return "Arm + Gripper\nTranslation Only\nConstraint: None\nCollidables: Self + Tool"
		}
		return "Arm Only\nTranslation Only\nConstraint: None\nCollidables: Self Only"
	case "random_rotation":
		if hasGripper {
			return "Arm + Gripper\nRotation Only\nConstraint: None\nCollidables: Self + Tool"
		}
		return "Arm Only\nRotation Only\nConstraint: None\nCollidables: Self Only"
	case "random_translation_linear", "linear_constraint":
		if hasGripper {
			return "Arm + Gripper\nTranslation\nConstraint: Linear\nCollidables: Self + Tool"
		}
		return "Arm Only\nTranslation\nConstraint: Linear\nCollidables: Self Only"
	case "random_rotation_linear":
		if hasGripper {
			return "Arm + Gripper\nRotation\nConstraint: Linear\nCollidables: Self + Tool"
		}
		return "Arm Only\nRotation\nConstraint: Linear\nCollidables: Self Only"
	}
	// Fallback: use the raw key.
	return strings.ReplaceAll(scenarioKey, "_", " ")
}
