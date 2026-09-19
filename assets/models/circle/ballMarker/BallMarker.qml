import QtQuick
import QtQuick3D

Node {
    id: ballMarker_obj

    Model {
        id: ballMarker
        source: "meshes/ballMarker.mesh"

        DefaultMaterial {
            id: defaultMaterial_material
            diffuseColor: "#ff999999"
        }
        materials: [
            defaultMaterial_material
        ]
    }
}
