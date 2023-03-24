#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <random>
#include <GL/glut.h>

const double CHARGE = -1.6e-19;
const double MASS = 9.1e-31;
const double EPSILON = 8.85e-12;

const double SPHERE_RADIUS = 1.0;
const double TIME_STEP = 1e-9;
const int NUM_ELECTRONS = 100;

std::mt19937_64 rng(std::chrono::high_resolution_clock::now().time_since_epoch().count());
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

double magnitude(const std::vector<double>& v) {
    double sum = 0;
    for (int i = 0; i < v.size(); i++) {
        sum += v[i] * v[i];
    }
    return std::sqrt(sum);
}

std::vector<double> normalize(const std::vector<double>& v) {
    double mag = magnitude(v);
    std::vector<double> result(v.size());
    for (int i = 0; i < v.size(); i++) {
        result[i] = v[i] / mag;
    }
    return result;
}

std::vector<double> reflect(const std::vector<double>& vector, const std::vector<double>& surface_normal) {
    double dot_prod = 0;
    for (int i = 0; i < vector.size(); i++) {
        dot_prod += vector[i] * surface_normal[i];
    }
    std::vector<double> reflected(vector.size());
    for (int i = 0; i < vector.size(); i++) {
        reflected[i] = vector[i] - 2 * dot_prod * surface_normal[i];
    }
    return reflected;
}

std::vector<double> random_vector(double min, double max, int size) {
    std::vector<double> result(size);
    for (int i = 0; i < size; i++) {
        result[i] = uniform(rng) * (max - min) + min;
    }
    return result;
}

std::vector<std::vector<double>> position(NUM_ELECTRONS);
std::vector<std::vector<double>> velocity(NUM_ELECTRONS);

void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    glColor3f(1.0, 1.0, 1.0);
    glutWireSphere(SPHERE_RADIUS, 20, 20);

    glPointSize(2.0);
    glBegin(GL_POINTS);
    for (int i = 0; i < NUM_ELECTRONS; i++) {
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(position[i][0], position[i][1], position[i][2]);
    }
    glEnd();

    glutSwapBuffers();
}

void idle() {
    std::vector<std::vector<double>> force(NUM_ELECTRONS, std::vector<double>(3, 0));
    for (int i = 0; i < NUM_ELECTRONS; i++) {
        for (int j = i+1; j < NUM
        std::vector<double> r(3);
        for (int k = 0; k < 3; k++) {
            r[k] = position[i][k] - position[j][k];
        }
        double dist = magnitude(r);
        double F = (CHARGE * CHARGE) / (4 * M_PI * EPSILON * dist * dist);
        for (int k = 0; k < 3; k++) {
            force[i][k] += F * r[k] / dist;
            force[j][k] -= F * r[k] / dist;
        }
    }
    for (int i = 0; i < NUM_ELECTRONS; i++) {
        std::vector<double> acceleration(3);
        for (int j = 0; j < 3; j++) {
            acceleration[j] = force[i][j] / MASS;
        }
        for (int j = 0; j < 3; j++) {
            velocity[i][j] += acceleration[j] * TIME_STEP;
            position[i][j] += velocity[i][j] * TIME_STEP;
        }
        double r = magnitude(position[i]);
        if (r > SPHERE_RADIUS) {
            std::vector<double> surface_normal = normalize(position[i]);
            velocity[i] = reflect(velocity[i], surface_normal);
            position[i] = SPHERE_RADIUS * normalize(position[i]);
        }
    }

    glutPostRedisplay();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Electrons on a Sphere");

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1.0, 0.1, 100.0);

    for (int i = 0; i < NUM_ELECTRONS; i++) {
        position[i] = random_vector(-SPHERE_RADIUS, SPHERE_RADIUS, 3);
        velocity[i] = random_vector(-1e6, 1e6, 3);
    }

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutMainLoop();

    return 0;
}
