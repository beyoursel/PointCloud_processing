#include "Bspline.h"


bspSurface::bspSurface(const vector<vector<Eigen::Vector3d>>& cnPoint, 
            const vector<float>& knots_u, 
            vector<float>& knots_v) 
{
    m_cnPoint = cnPoint;
    m_knots_u = knots_u; // u节点向量
    m_knots_v = knots_v;

    m_nu = m_cnPoint.size() - 1; // m_cnPoint.size()获得控制点行数
    m_nv = m_cnPoint[0].size() - 1; // m_cnPoint[0].size()获得控制点的列数
    m_ku = m_knots_u.size() - 1 - m_nu; // m = n + p + 1 m+1: 节点个数; n+1：控制点个数; p: 阶数
    m_kv = m_knots_v.size() - 1 - m_nv;
}

// constructor
bspSurface::bspSurface(const bspSurface& surface) {
    m_cnPoint = surface.m_cnPoint;
    m_knots_u = surface.m_knots_u;
    m_knots_v = surface.m_knots_v;

    m_nu = surface.m_nu;
    m_nv = surface.m_nv;
    m_ku = surface.m_kv;
    m_kv = surface.m_kv; 
}

// 函数运算符重载
bspSurface& bspSurface::operator=(const bspSurface& surface) {
    m_cnPoint = surface.m_cnPoint;
    m_knots_u = surface.m_knots_u;
    m_knots_v = surface.m_knots_v;

    m_nu = surface.m_nu;
    m_nv = surface.m_nv;
    m_ku = surface.m_ku;
    m_kv = surface.m_kv;
    return *this;
}

// 根据参数u,v计算曲面上的坐标
Eigen::Vector3d bspSurface::calPos(const float& u, const float& v) {

    vector<Eigen::Vector3d> v_constant(m_nu + 1);
    for (int i = 0; i < v_constant.size(); ++i)
    {
        v_constant[i] = calPos(m_cnPoint[i], m_knots_v, v);
    }
    return calPos(v_constant, m_knots_u, u);
}

Eigen::Vector3d bspSurface::calPos(const vector<Eigen::Vector3d>& controlpoint, const vector<float>& knots, const float& t)
{
    int n = controlpoint.size() - 1;
    int k = knots.size() - controlpoint.size(); // 阶数
    int L = 0;
    // 计算t所处的区间[t_L, t_(L+1)], t只在[knots[k-1], knots[n+1]]中有效
    if (t >= knots[n+1])
    {
        L = n;
    } else if (t <= knots[k-1])
    {
        L = k - 1;
    }
    else
    {
        for (int i = k - 1; i <= n + 1; ++i)
        {
            if (t >= knots[i] && t<knots[i+1])
            {
                L = i;
                break;
            }
        }
    }

    if (L >= n + 1) L = n;

    vector<Eigen::Vector3d> temp(k);
    for (int i = 0; i < k; ++i) {
        temp[i] = controlpoint[i + L - k + 1];
    }

    //de-BoorCox算法
    for (int r = 1; r <= k - 1; ++r)
    {
        for (int i = 0; i <= k - r - 1; ++i)
        {
            int index = L - k + 1 + r;
            double factor = 0;
            if (knots[index + i + k - r] != knots[index + i])
            {
                factor = (t - knots[index + i]) / (knots[index + i + k - r] - knots[index + i]);
            }
            temp[i] = factor*temp[i + 1] + (1 - factor)*temp[i];

        }
    }
    return temp[0];

}



void bspSurface::getFittingSurface(
    vector<Eigen::Vector3d>& vertices, float step)
{

    int m = static_cast<int>((m_knots_u[m_nu + 1] - m_knots_u[m_ku - 1]) / step);
    int n = static_cast<int>((m_knots_v[m_nv + 1] - m_knots_v[m_kv - 1]) / step);

    for (int i = 0; i <= m; ++i)
    {
        for (int j = 0; j <= n; ++j)
        {
            float u = 0, v = 0;
            if (i == m)
            {
                u = m_knots_u[m_nu + 1];
                v = m_knots_v[m_kv - 1] + j*step;
                
            }
            else if (j == n)
            {
                u = m_knots_u[m_ku - 1] + i*step;
                v = m_knots_v[m_nv + 1];
                
            }
            else
            {
                u = m_knots_u[m_ku - 1] + i*step;
                v = m_knots_v[m_kv - 1] + j*step;
            }
            
            Eigen::Vector3d temp = calPos(u, v);
            vertices.push_back(temp);
        }
    }
}

//根据精度生成vbo,vao,ebo，用于绘制
void bspSurface::getbuffer_object(
    vector<Eigen::Vector3d>& vertices, vector<Eigen::Vector3d>& normals,
    vector<unsigned short>& edge_indices, vector<unsigned short>& face_indices,float step)
{

    int m = static_cast<int>((m_knots_u[m_nu + 1] - m_knots_u[m_ku - 1]) / step);
    int n = static_cast<int>((m_knots_v[m_nv + 1] - m_knots_v[m_kv - 1]) / step);

    for (int i = 0; i <= m; ++i)
    {
        for (int j = 0; j <= n; ++j)
        {
            float u = 0, v = 0;
            if (i == m)
            {
                u = m_knots_u[m_nu + 1];
                v = m_knots_v[m_kv - 1] + j*step;
                
            }
            else if (j == n)
            {
                u = m_knots_u[m_ku - 1] + i*step;
                v = m_knots_v[m_nv + 1];
                
            }
            else
            {
                u = m_knots_u[m_ku - 1] + i*step;
                v = m_knots_v[m_kv - 1] + j*step;
            }
            
            Eigen::Vector3d temp = calPos(u, v);
            vertices.push_back(temp);
        }
    }
    normals.resize(vertices.size());
    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            Eigen::Vector3d vector1 = vertices[i*(n + 1) + j + 1] - vertices[i*(n + 1) + j];
            Eigen::Vector3d vector2 = vertices[i*(n + 1) + j + (n + 1)] - vertices[i*(n + 1) + j];
            Eigen::Vector3d temp_normal = vector2.cross(vector1);

            temp_normal.normalize();
            normals[i*(n + 1) + j] = temp_normal;

        }
    }
    for (int i = 0; i <= m; ++i)
    {
        normals[i*(n + 1) + n] = normals[i*(n + 1) + n - 1];
    }
    for (int j = 0; j <= n; ++j)
    {
        normals[m*(n + 1) + j] = normals[(m - 1)*(n + 1) + j];
    }
    
    for (unsigned short i = 0; i < m; ++i)
    {
        for (unsigned short j = 0; j < n; ++j)
        {
            face_indices.push_back(i*(n + 1) + j);
            face_indices.push_back(i*(n + 1) + j + 1);
            face_indices.push_back(i*(n + 1) + j + (n + 1));
            
            face_indices.push_back(i*(n + 1) + j + (n + 1));
            face_indices.push_back(i*(n + 1) + j + 1);
            face_indices.push_back(i*(n + 1) + j + (n + 1) + 1);

            edge_indices.push_back(i*(n + 1) + j);
            edge_indices.push_back(i*(n + 1) + j + 1);
            edge_indices.push_back(i*(n + 1) + j);
            edge_indices.push_back(i*(n + 1) + j + (n + 1));
        
        }
    }

    for (unsigned short i = 0; i < m; ++i)
    {
        edge_indices.push_back(i*(n + 1) + n);
        edge_indices.push_back((i + 1)*(n + 1) + n);
    }
    for (unsigned short j = 0; j < n; ++j)
    {
        edge_indices.push_back(m*(n + 1) + j);
        edge_indices.push_back(m*(n + 1) + j + 1);
    }
}


//生成控制顶点的顶点及边上点的索引，用于绘制
void bspSurface::getcontrolpoint(vector<Eigen::Vector3d>& vertices, vector<unsigned short>& edge_indices)
{
    for (int i = 0; i <= m_nu; ++i)
    {
        for (int j = 0; j <= m_nv; ++j)
        {
            vertices.push_back(m_cnPoint[i][j]);
        }
    }
    for (unsigned short i = 0; i < m_nu; ++i)
    {
        for (unsigned short j = 0; j < m_nv; ++j)
        {

            edge_indices.push_back(i*(m_nv + 1) + j);
            edge_indices.push_back(i*(m_nv + 1) + j + 1);
            edge_indices.push_back(i*(m_nv + 1) + j);
            edge_indices.push_back(i*(m_nv + 1) + j + (m_nv + 1));

        }
    }

    for (unsigned short i = 0; i < m_nu; ++i)
    {
        edge_indices.push_back(i*(m_nv + 1) + m_nv);
        edge_indices.push_back((i + 1)*(m_nv + 1) + m_nv);
    }
    for (unsigned short j = 0; j < m_nv; ++j)
    {
        edge_indices.push_back(m_nu*(m_nv + 1) + j);
        edge_indices.push_back(m_nu*(m_nv + 1) + j + 1);
    }
}

