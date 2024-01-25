using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Priority_Queue;
using System.Linq;

public class Search : MonoBehaviour
{
    public static bool Dijkstra(AINavNode source, AINavNode destination, ref List<AINavNode> path, int maxSteps)
    {
        bool found = false;

        // create priority queue
        var nodes = new SimplePriorityQueue<AINavNode>();

        // set source cost to 0
        source.Cost = 0;
        // enqueue source node with the course cost as the priority
        nodes.EnqueueWithoutDuplicates(source, source.Cost);

        // set the current number of steps
        int steps = 0;
        while (!found && nodes.Count > 0 && steps < maxSteps)
        {
            // dequeue node
            var node = nodes.Dequeue();

            // check if node is the destination node
            if (node == destination)
            {
                // set found to true
                found = true;
                // break
                break;
            }

            // for each neighbor of node
            foreach (var neighbor in node.neighbors)
            {
                // calculate cost to neighbor = node cost + distance to neighbor
                float cost = node.Cost + Vector3.Distance(node.transform.position, neighbor.transform.position);
                // if cost is less than neighbor cost
                if (cost < neighbor.Cost)
                {
                    // set neighbor cost to cost
                    neighbor.Cost = cost;
                    // set neighbor parent to node
                    neighbor.Parent = node;

                    // enqueue neighbor with the cost as the priority
                    nodes.EnqueueWithoutDuplicates(neighbor, neighbor.Cost);
                }
            }

            if (found)
            {
                // create path from destination to course using node parents
                path = new List<AINavNode>();
                CreatePathFromParents(destination, ref path);
            }
            else
            {
                path = nodes.ToList();
            }

        }
        
        return found;
    }

    public static bool AStar(AINavNode source, AINavNode destination, ref List<AINavNode> path, int maxSteps)
    {
        bool found = false;

        // create priority queue
        var nodes = new SimplePriorityQueue<AINavNode>();

        // set source cost to 0
        source.Cost = 0;
        // set heuristic to the distance of source to the destination
        float heuristic = Vector3.Distance(source.transform.position, destination.transform.position);
        // enqueue source node with the source cost + heuristic as the priority
        nodes.EnqueueWithoutDuplicates(source, source.Cost + heuristic);

        // set the current number of steps
        int steps = 0;
        while (!found && nodes.Count > 0 && steps < maxSteps)
        {
            // dequeue node
            var node = nodes.Dequeue();

            // check if node is the destination node
            if (node == destination)
            {
                // set found to true
                found = true;
                // break
                break;
            }

            // for each neighbor of node
            foreach (var neighbor in node.neighbors)
            {
                // calculate cost to neighbor = node cost + distance to neighbor
                float cost = node.Cost + Vector3.Distance(node.transform.position, neighbor.transform.position);
                // if cost is less than neighbor cost
                if (cost < neighbor.Cost)
                {
                    // set neighbor cost to cost
                    neighbor.Cost = cost;
                    // set neighbor parent to node
                    neighbor.Parent = node;

                    // calculate heuristic = distance from neighbor to destination
                    heuristic = Vector3.Distance(neighbor.transform.position, destination.transform.position);

                    // enqueue without duplicates, neighbor cost + heuristic as priority
                    // the closer the neighbor to the destination, the higher the priority
                    nodes.EnqueueWithoutDuplicates(neighbor, cost + heuristic);
                }
            }

            if (found)
            {
                // create path from destination to course using node parents
                path = new List<AINavNode>();
                CreatePathFromParents(destination, ref path);
            }
            else
            {
                path = nodes.ToList();
            }

        }
        
        return found;
    }

    public static void CreatePathFromParents(AINavNode node, ref List<AINavNode> path)
    {
        // while node not null
        while (node != null)
        {
            // add node to list path
            path.Add(node);
            // set node to node parent
            node = node.Parent;
        }

        // reverse path
        path.Reverse();
    }
};