using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Kim : CharacterController
{
    [SerializeField] float ContextRadius = 10f;
    [SerializeField] float ZombieDangerRadius = 5f;
    [SerializeField] float PathUpdateInterval = 0.5f;

    private float pathUpdateTimer = 0f;
    private List<Grid.Tile> currentPath = new List<Grid.Tile>();
    private KimState currentState = KimState.SeekingFinish;
    private Zombie closestZombie = null;

    private enum KimState
    {
        SeekingFinish,
        FleeingFromZombie
    }

    public override void StartCharacter()
    {
        base.StartCharacter();
        pathUpdateTimer = 0f;
    }

    public override void UpdateCharacter()
    {
        base.UpdateCharacter();
        pathUpdateTimer -= Time.deltaTime;
        if (pathUpdateTimer <= 0f) {
            pathUpdateTimer = PathUpdateInterval;
            UpdateStateMachine();
        }
    }

    void UpdateStateMachine()
    {
        closestZombie = GetClosest(GetContextByTag("Zombie"))?.GetComponent<Zombie>();
        if (closestZombie != null) {
            float distanceToZombie = Vector3.Distance(transform.position, closestZombie.transform.position);
            if (distanceToZombie < ZombieDangerRadius) currentState = KimState.FleeingFromZombie;
            else currentState = KimState.SeekingFinish;
        } else currentState = KimState.SeekingFinish;

        switch (currentState) {
            case KimState.SeekingFinish:
                ExecuteSeekingFinish();
                break;
            case KimState.FleeingFromZombie:
                ExecuteFleeingFromZombie();
                break;
        }
    }

    void ExecuteSeekingFinish()
    {
        Grid.Tile targetTile = Grid.Instance.GetFinishTile();
        if (targetTile != null && myCurrentTile != null) {
            currentPath = FindPathAStar(myCurrentTile, targetTile);
            if (currentPath != null && currentPath.Count > 0) SetWalkBuffer(currentPath);
        }
    }

    void ExecuteFleeingFromZombie()
    {
        if (closestZombie == null || myCurrentTile == null) return;
        Grid.Tile zombieTile = Grid.Instance.GetClosest(closestZombie.transform.position);
        Grid.Tile safeTile = FindSafeTile(zombieTile);
        if (safeTile != null) {
            currentPath = FindPathAStar(myCurrentTile, safeTile);
            if (currentPath != null && currentPath.Count > 0) SetWalkBuffer(currentPath);
        }
    }

    Grid.Tile FindSafeTile(Grid.Tile dangerTile) {
        Grid.Tile bestTile = null;
        float maxDistance = 0f;
        List<Grid.Tile> allTiles = Grid.Instance.GetTiles();
        Vector3 dangerPos = Grid.Instance.WorldPos(dangerTile);
        foreach (Grid.Tile tile in allTiles) {
            if (tile.occupied) continue;
            Vector3 tilePos = Grid.Instance.WorldPos(tile);
            float distFromDanger = Vector3.Distance(tilePos, dangerPos);
            float distFromKim = Vector3.Distance(tilePos, transform.position);
            if (distFromDanger > ZombieDangerRadius && distFromDanger > maxDistance && distFromKim < ContextRadius) {
                maxDistance = distFromDanger;
                bestTile = tile;
            }
        }
        if (bestTile == null) {
            Vector3 fleeDirection = (transform.position - dangerPos).normalized;
            Vector3 fleeTarget = transform.position + fleeDirection * 5f;
            bestTile = Grid.Instance.GetClosest(fleeTarget);
        }
        return bestTile;
    }

    // A*
    List<Grid.Tile> FindPathAStar(Grid.Tile start, Grid.Tile goal)
    {
        if (start == null || goal == null) return new List<Grid.Tile>();
        List<AStarNode> openList = new List<AStarNode>();
        HashSet<Grid.Tile> closedSet = new HashSet<Grid.Tile>();
        Dictionary<Grid.Tile, AStarNode> allNodes = new Dictionary<Grid.Tile, AStarNode>();
        AStarNode startNode = new AStarNode(start, null, 0, GetHeuristic(start, goal));
        openList.Add(startNode);
        allNodes[start] = startNode;
        while (openList.Count > 0) {
            AStarNode currentNode = GetLowestFCostNode(openList);
            openList.Remove(currentNode);
            if (Grid.Instance.IsSameTile(currentNode.tile, goal)) return ReconstructPath(currentNode);
            closedSet.Add(currentNode.tile);
            List<Grid.Tile> neighbors = GetNeighbors(currentNode.tile);
            foreach (Grid.Tile neighborTile in neighbors) {
                if (closedSet.Contains(neighborTile) || neighborTile.occupied) continue;
                float tentativeGCost = currentNode.gCost + GetDistance(currentNode.tile, neighborTile);
                AStarNode neighborNode;
                if (!allNodes.ContainsKey(neighborTile)) {
                    float hCost = GetHeuristic(neighborTile, goal);
                    neighborNode = new AStarNode(neighborTile, currentNode, tentativeGCost, hCost);
                    allNodes[neighborTile] = neighborNode;
                    openList.Add(neighborNode);
                } else {
                    neighborNode = allNodes[neighborTile];
                    if (tentativeGCost < neighborNode.gCost) {
                        neighborNode.gCost = tentativeGCost;
                        neighborNode.parent = currentNode;
                    }
                }
            }
        }
        return new List<Grid.Tile>();
    }

    List<Grid.Tile> GetNeighbors(Grid.Tile tile) {
        List<Grid.Tile> neighbors = new List<Grid.Tile>();
        Vector2Int[] directions = new Vector2Int[] {
            new Vector2Int(1, 0),   // Right
            new Vector2Int(-1, 0),  // Left
            new Vector2Int(0, 1),   // Up
            new Vector2Int(0, -1),  // Down
            new Vector2Int(1, 1),   // Up-Right
            new Vector2Int(1, -1),  // Down-Right
            new Vector2Int(-1, 1),  // Up-Left
            new Vector2Int(-1, -1)  // Down-Left
        };
        foreach (Vector2Int dir in directions) {
            Grid.Tile neighbor = Grid.Instance.TryGetTile(new Vector2Int(tile.x + dir.x, tile.y + dir.y));
            if (neighbor != null && !neighbor.occupied) neighbors.Add(neighbor);
        }
        return neighbors;
    }

    AStarNode GetLowestFCostNode(List<AStarNode> openList) {
        AStarNode lowestNode = openList[0];
        foreach (AStarNode node in openList) {
            if (node.fCost < lowestNode.fCost || (node.fCost == lowestNode.fCost && node.hCost < lowestNode.hCost)) lowestNode = node;
        }
        return lowestNode;
    }

    float GetHeuristic(Grid.Tile a, Grid.Tile b) {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        return (dx + dy) + (Mathf.Sqrt(2) - 2) * Mathf.Min(dx, dy);
    }

    float GetDistance(Grid.Tile a, Grid.Tile b) {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        if (dx == 1 && dy == 1) return Mathf.Sqrt(2);
        return 1f;
    }

    List<Grid.Tile> ReconstructPath(AStarNode goalNode) {
        List<Grid.Tile> path = new List<Grid.Tile>();
        AStarNode currentNode = goalNode;
        while (currentNode != null) {
            path.Add(currentNode.tile);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        return path;
    }

    private class AStarNode {
        public Grid.Tile tile;
        public AStarNode parent;
        public float gCost;
        public float hCost;
        public float fCost { get { return gCost + hCost; } }

        public AStarNode(Grid.Tile tile, AStarNode parent, float gCost, float hCost) {
            this.tile = tile;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
        }
    }

    Vector3 GetEndPoint() {
        return Grid.Instance.WorldPos(Grid.Instance.GetFinishTile());
    }

    GameObject[] GetContextByTag(string aTag) {
        Collider[] context = Physics.OverlapSphere(transform.position, ContextRadius);
        List<GameObject> returnContext = new List<GameObject>();
        foreach (Collider c in context) if (c.transform.CompareTag(aTag)) returnContext.Add(c.gameObject);
        return returnContext.ToArray();
    }

    GameObject GetClosest(GameObject[] aContext) {
        float dist = float.MaxValue;
        GameObject Closest = null;
        foreach (GameObject z in aContext) {
            float curDist = Vector3.Distance(transform.position, z.transform.position);
            if (curDist < dist) {
                dist = curDist;
                Closest = z;
            }
        }
        return Closest;
    }

    private void OnDrawGizmos()
    {
        if (currentPath == null || currentPath.Count == 0) return;
        Color pathColor = currentState == KimState.FleeingFromZombie ? Color.red : Color.green;
        for (int i = 0; i < currentPath.Count; i++) {
            Vector3 position = Grid.Instance.WorldPos(currentPath[i]);
            Gizmos.color = pathColor;
            Gizmos.DrawWireSphere(position, 0.3f);
            if (i < currentPath.Count - 1) {
                Vector3 nextPosition = Grid.Instance.WorldPos(currentPath[i + 1]);
                Gizmos.DrawLine(position, nextPosition);
            }
        }
        if (closestZombie != null) {
            Gizmos.color = new Color(1f, 0f, 0f, 0.2f);
            Gizmos.DrawWireSphere(closestZombie.transform.position, ZombieDangerRadius);
        }
        Gizmos.color = new Color(0f, 0f, 1f, 0.2f);
        Gizmos.DrawWireSphere(transform.position, ContextRadius);
    }
}
