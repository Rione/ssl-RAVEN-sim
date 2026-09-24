import QtQuick 2.15
import QtQuick3D

Node {
    id: frame_obj

    Model {
        id: centerRight
        source: "meshes/centerRight.mesh"

        DefaultMaterial {
            id: opaque_63_63_63__material
            diffuseColor: "#ff3f3f3f"
        }
        materials: [
            opaque_63_63_63__material
        ]
    }

    Model {
        id: centerLeft
        source: "meshes/centerLeft.mesh"
        materials: [
            opaque_63_63_63__material
        ]
    }

    Model {
        id: body
        source: "meshes/body.mesh"
        materials: [
            opaque_63_63_63__material
        ]
    }

    Model {
        id: dribbler
        source: "meshes/dribbler.mesh"

        DefaultMaterial {
            id: _________material
            diffuseColor: "#ff272727"
        }
        materials: [
            _________material
        ]
    }

    Model {
        id: chip
        source: "meshes/chip.mesh"

        DefaultMaterial {
            id: ______________material
            diffuseColor: "#fff5f5f6"
        }
        materials: [
            ______________material
        ]
    }
}
