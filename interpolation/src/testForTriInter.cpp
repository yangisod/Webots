#include <Eigen/Dense>
#include <vector>
#include <iostream>
using namespace std;

struct xy
{
    int x;
    int y;
};
int main(int argc,char **argv)
{
    vector<xy> data{xy{0, 0}, xy{2, 2}, xy{0, 4}};
    xy test{2, 3};
    test.x = atoi(argv[1]);
    test.y = atoi(argv[2]);
    Eigen::Vector2d ab, ac, ap;
    ap << test.x - data[0].x, test.y - data[0].y;
    // ap = Eigen::Vector2d(test.x - data[0].x, test.y - data[0].y);

    ab = Eigen::Vector2d(data[1].x - data[0].x, data[1].y - data[0].y);
    ac = Eigen::Vector2d(data[2].x - data[0].x, data[2].y - data[0].y);

    // ap = x * ab + y * ac;
    Eigen::Matrix2d A;
    A << data[1].x - data[0].x, data[2].x - data[0].x, data[1].y - data[0].y, data[2].y - data[0].y;
    cout << A << endl
         << ap << endl;
    auto x = A.colPivHouseholderQr().solve(ap); //求解Ax=b

    cout<<x<<endl;
    return 0;
}