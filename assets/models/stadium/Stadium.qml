import QtQuick 2.15
import QtQuick3D

Node {
    id: stadium_obj

    Model {
        id: stadium
        source: "meshes/stadium.mesh"

        DefaultMaterial {
            id: __________material
            diffuseColor: "#ff191919"
        }

        DefaultMaterial {
            id: __________________material
            diffuseColor: "#ff595959"
        }

        DefaultMaterial {
            id: _________material
            diffuseColor: "#ff191919"
        }

        DefaultMaterial {
            id: ______________material
            diffuseColor: "#fff5f5f6"
        }
        materials: [
            __________material,
            __________________material,
            _________material,
            ______________material
        ]
    }
}
