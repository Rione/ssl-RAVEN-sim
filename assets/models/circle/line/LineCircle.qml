import QtQuick
import QtQuick3D

Node {
    id: node

    property color lineColor: "#fff5f5f6"

    // Resources
    PrincipledMaterial {
        id: __________________material
        objectName: "アルミニウム_-_ビーズ_ブラスト"
        baseColor: node.lineColor
        indexOfRefraction: 1
    }

    // Nodes:
    Node {
        id: lineCircle_obj
        objectName: "lineCircle.obj"
        Model {
            id: lineCircle
            objectName: "lineCircle"
            source: "meshes/lineCircle_mesh.mesh"
            materials: [
                __________________material
            ]
        }
    }

    // Animations:
}
