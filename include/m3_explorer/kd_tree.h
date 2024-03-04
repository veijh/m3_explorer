#ifndef KD_TREE_H
#define KD_TREE_H
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
using namespace std;

class Point
{
    public:
        Point(){}
        float x;
        float y;
        float z;
        friend bool XSortFunc(Point& a, Point& b);
        friend bool YSortFunc(Point& a, Point& b);
        friend bool ZSortFunc(Point& a, Point& b);
        Point operator=(const Point& other);
        bool operator==(const Point& other);
        bool operator!=(const Point& other);
};
 
class KdTree
{
    public:
        enum SPLIT_DIM {SPLIT_NULL, SPLIT_X, SPLIT_Y, SPLIT_Z};
        KdTree(){splitD = SPLIT_NULL; splitV = 0; leftTree = nullptr, rightTree = nullptr; isVisited = false;}
        ~KdTree();
        KdTree(Point point, KdTree* leftTree, KdTree* rightTree, SPLIT_DIM splitD, float splitV)
        {
            this->point = point;
            this->leftTree = leftTree;
            this->rightTree = rightTree;
            this->splitD = splitD;
            this->splitV = splitV;
            isVisited = false;
        }
        Point point;
        SPLIT_DIM splitD;
        float splitV;
        int depth;
        bool isVisited;
        KdTree* leftTree;
        KdTree* rightTree;
        bool isLeftTreeNode(Point point);
        bool isRightTreeNode(Point point);
        void BuildTree(vector<Point>& pointArray, int l, int r);
        void Insert(Point point);
        void Delete(KdTree* parent, Point point);
        KdTree* GetMaxXTreeNode();
        KdTree* GetMinXTreeNode();
        KdTree* GetMaxYTreeNode();
        KdTree* GetMinYTreeNode();
        KdTree* GetMaxZTreeNode();
        KdTree* GetMinZTreeNode();
        friend void GetNearestPoint(KdTree* root, KdTree* parent, Point point, float& minDis, Point& tarPoint);
};
#endif
