import QtQuick 2.15
import QtQuick3D

Node {
    id: ball_obj

    Model {
        id: ball
        source: "meshes/ball.mesh"

        DefaultMaterial {
            id: material_001_material
            diffuseColor: "#ffcc8e02"
        }
        materials: [
            material_001_material
        ]
    }
}
