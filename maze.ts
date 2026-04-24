// 定义坐标点的接口
export interface Point {
    row: number;
    col: number;
}

/**
 * 使用广度优先搜索 (BFS) 在迷宫中寻找最短路径
 * * @param maze 二维矩阵，0 表示互通/可走，1 表示障碍物
 * @param start 起点坐标
 * @param end 终点坐标
 * @returns 返回从起点到终点的最短路径数组；如果无解则返回 null
 */
export function findShortestPath(maze: number[][], start: Point, end: Point): Point[] | null {
    const rows = maze.length;
    if (rows === 0) return null;
    const cols = maze[0].length;

    // 边界检查与合法性校验：起点或终点在网格外部，或者本身就是障碍物
    if (
        start.row < 0 || start.row >= rows || start.col < 0 || start.col >= cols ||
        end.row < 0 || end.row >= rows || end.col < 0 || end.col >= cols ||
        maze[start.row][start.col] !== 0 || maze[end.row][end.col] !== 0
    ) {
        return null;
    }

    // 队列用于 BFS 遍历
    const queue: Point[] = [start];

    // 记录已经访问过的节点，避免走回头路和死循环
    const visited: boolean[][] = Array.from({ length: rows }, () => Array(cols).fill(false));
    visited[start.row][start.col] = true;

    // 记录路径：parentMap 记录每个节点是从哪个相邻节点走过来的
    // 使用二维数组存放父节点引用
    const parent: (Point | null)[][] = Array.from({ length: rows }, () => Array(cols).fill(null));

    // 定义四个移动方向：上、下、左、右
    const directions = [
        { r: -1, c: 0 }, // 上
        { r: 1, c: 0 },  // 下
        { r: 0, c: -1 }, // 左
        { r: 0, c: 1 }   // 右
    ];

    let isReached = false;

    // 开始 BFS
    while (queue.length > 0) {
        // 出队当前节点
        const current = queue.shift()!;

        // 检查是否到达终点
        if (current.row === end.row && current.col === end.col) {
            isReached = true;
            break;
        }

        // 探索相邻的四个方向
        for (const dir of directions) {
            const nextRow = current.row + dir.r;
            const nextCol = current.col + dir.c;

            // 判断相邻节点是否在矩阵范围内，是否可通行，且尚未被访问过
            if (
                nextRow >= 0 && nextRow < rows &&
                nextCol >= 0 && nextCol < cols &&
                maze[nextRow][nextCol] === 0 &&
                !visited[nextRow][nextCol]
            ) {
                // 标记为已访问，并加入队列
                visited[nextRow][nextCol] = true;
                queue.push({ row: nextRow, col: nextCol });

                // 记录是从 current 节点走到 next 节点的
                parent[nextRow][nextCol] = current;
            }
        }
    }

    // 如果队列清空依然没有到达终点，说明没有连通的路径
    if (!isReached) {
        return null;
    }

    // 回溯生成最短路径
    const path: Point[] = [];
    let currentPathNode: Point | null = end;

    while (currentPathNode !== null) {
        path.push(currentPathNode);
        currentPathNode = parent[currentPathNode.row][currentPathNode.col];
    }

    // 因为我们是从终点往回追溯到起点的，所以需要反转数组
    return path.reverse();
}
