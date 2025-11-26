//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <queue>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    GreedyBFS,
    AStar
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
    };

    struct CompareEntry {
        bool operator()(Entry const& a, Entry const& b) const {
            return a.dist > b.dist; // min-heap
        }
    };

    void dijkstra(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        // TODO: Add your code here

        set_final_path(parent);
    }


    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        // TODO: Add your code here

        set_final_path(parent);
    }

    void greedy_bfs(Graph &graph) {
        std::unordered_map<Node*, Node*> parent;
        std::unordered_map<Node*, bool> visited;

        auto heuristic = [&](Node* a, Node* b){
            float dx = a->coord.x - b->coord.x;
            float dy = a->coord.y - b->coord.y;
            return std::sqrt(dx*dx + dy*dy);
        };

        std::priority_queue<Entry, std::vector<Entry>, CompareEntry> pq;
        pq.push({src, heuristic(src, dest)});
        parent[src] = nullptr;
        while (!pq.empty()) {
            Entry current = pq.top();
            pq.pop();
            if (visited[current.node]) { continue; }
            visited[current.node] = true;
            if (current.node == dest) { break; }
            for (Edge* edge : current.node->edges) {
                Node* next = (edge->src == current.node) ? edge->dest : edge->src;
                if (visited[next]) { continue; }
                visited_edges.emplace_back(
                        current.node->coord,
                        next->coord,
                        sf::Color::Yellow,
                        1.5f
                );
                parent[next] = current.node;
                pq.push({next, heuristic(next, dest)});
                render();
            }
        }
        set_final_path(parent);
    }


    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        sf::sleep(sf::milliseconds(10));
        window_manager->clear();
        for (sfLine &line : visited_edges) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }
        window_manager->display();
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node*, Node*>& parent) {
        path.clear();
        Node* current = dest;
        while (current && parent[current]) {
            Node* prev = parent[current];
            path.emplace_back(prev->coord, current->coord, sf::Color::Green, 2.0f);
            current = prev;
        }
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) { return; }
        path.clear();
        visited_edges.clear();
        switch (algorithm) {
            case Dijkstra: dijkstra(graph); break;
            case AStar: a_star(graph); break;
            case GreedyBFS: greedy_bfs(graph); break;
            default: break;
        }
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
