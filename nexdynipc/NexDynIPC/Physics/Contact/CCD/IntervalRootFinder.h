#pragma once

#include "NexDynIPC/Math/Interval.h"
#include <functional>
#include <stack>
#include <limits>

namespace NexDynIPC::Physics::CCD {

using namespace NexDynIPC::Math;

constexpr int INTERVAL_ROOT_FINDER_MAX_ITERATIONS = 1000;
constexpr double DEFAULT_TOI_TOLERANCE = 1e-8;

inline std::pair<Interval, Interval> bisect(const Interval& i) {
    double mid = midpoint(i);
    return { Interval(lower(i), mid), Interval(mid, upper(i)) };
}

inline bool intervalRootFinderImpl(
    const std::function<VectorXI(const VectorXI&)>& f,
    const std::function<bool(const VectorXI&)>& constraint_predicate,
    const std::function<bool(const VectorXI&)>& is_domain_valid,
    const VectorXI& x0,
    const Eigen::VectorXd& tol,
    VectorXI& x,
    int max_iterations) {
    
    VectorXI earliest_root = VectorXI::Constant(
        x0.size(), Interval(std::numeric_limits<double>::infinity()));
    bool found_root = false;

    std::stack<VectorXI> xs;
    xs.push(x0);

    VectorXI x_tol(tol.size());
    for (int i = 0; i < x_tol.size(); i++) {
        x_tol(i) = Interval(0, tol(i));
    }
    
    if (zero_in(f(x_tol))) {
        Eigen::VectorXd new_tol = tol;
        new_tol(0) /= 1e2;
        x_tol.resize(new_tol.size());
        for (int i = 0; i < x_tol.size(); i++) {
            x_tol(i) = Interval(0, new_tol(i));
        }
    }

    for (int iter = 0; !xs.empty() && iter < max_iterations; ++iter) {
        x = xs.top();
        xs.pop();

        if (x(0).lower() >= earliest_root(0).lower()) {
            continue;
        }

        if (!is_domain_valid(x)) {
            continue;
        }

        if (!constraint_predicate(x)) {
            continue;
        }

        VectorXI y = f(x);

        if (!zero_in(y)) {
            continue;
        }

        bool all_within_tol = true;
        for (int i = 0; i < x.size(); i++) {
            if (width(x(i)) > tol(i)) {
                all_within_tol = false;
                break;
            }
        }

        if (all_within_tol) {
            if (x(0).lower() < earliest_root(0).lower()) {
                earliest_root = x;
                found_root = true;
            }
            continue;
        }

        int split_dim = 0;
        double max_width = width(x(0));
        for (int i = 1; i < x.size(); i++) {
            double w = width(x(i));
            if (w > max_width) {
                max_width = w;
                split_dim = i;
            }
        }

        auto [lo, hi] = bisect(x(split_dim));
        
        VectorXI x_lo = x;
        x_lo(split_dim) = lo;
        xs.push(x_lo);

        VectorXI x_hi = x;
        x_hi(split_dim) = hi;
        xs.push(x_hi);
    }

    if (found_root) {
        x = earliest_root;
    }
    
    return found_root;
}

inline bool intervalRootFinder(
    const std::function<VectorXI(const VectorXI&)>& f,
    const std::function<bool(const VectorXI&)>& constraint_predicate,
    const std::function<bool(const VectorXI&)>& is_domain_valid,
    const VectorXI& x0,
    const Eigen::VectorXd& tol,
    VectorXI& x,
    int max_iterations = INTERVAL_ROOT_FINDER_MAX_ITERATIONS) {
    
    return intervalRootFinderImpl(f, constraint_predicate, is_domain_valid, x0, tol, x, max_iterations);
}

inline bool intervalRootFinder(
    const std::function<VectorXI(const VectorXI&)>& f,
    const std::function<bool(const VectorXI&)>& is_domain_valid,
    const VectorXI& x0,
    const Eigen::VectorXd& tol,
    VectorXI& x,
    int max_iterations = INTERVAL_ROOT_FINDER_MAX_ITERATIONS) {
    
    return intervalRootFinderImpl(
        f, 
        [](const VectorXI&) { return true; }, 
        is_domain_valid, x0, tol, x, max_iterations);
}

inline bool intervalRootFinder(
    const std::function<VectorXI(const VectorXI&)>& f,
    const VectorXI& x0,
    const Eigen::VectorXd& tol,
    VectorXI& x,
    int max_iterations = INTERVAL_ROOT_FINDER_MAX_ITERATIONS) {
    
    return intervalRootFinderImpl(
        f, 
        [](const VectorXI&) { return true; }, 
        [](const VectorXI&) { return true; }, 
        x0, tol, x, max_iterations);
}

inline bool intervalRootFinder(
    const std::function<Interval(const Interval&)>& f,
    const std::function<bool(const Interval&)>& constraint_predicate,
    const Interval& x0,
    double tol,
    Interval& x,
    int max_iterations = INTERVAL_ROOT_FINDER_MAX_ITERATIONS) {
    
    VectorXI x0_vec = VectorXI::Constant(1, x0);
    VectorXI x_vec;
    Eigen::VectorXd tol_vec = Eigen::VectorXd::Constant(1, tol);
    
    bool found_root = intervalRootFinderImpl(
        [&](const VectorXI& x) {
            return VectorXI::Constant(1, f(x(0)));
        },
        [&](const VectorXI& x) {
            return constraint_predicate(x(0));
        },
        [&](const VectorXI&) { return true; },
        x0_vec, tol_vec, x_vec, max_iterations);
    
    if (found_root) {
        x = x_vec(0);
    }
    return found_root;
}

inline bool intervalRootFinder(
    const std::function<Interval(const Interval&)>& f,
    const Interval& x0,
    double tol,
    Interval& x,
    int max_iterations = INTERVAL_ROOT_FINDER_MAX_ITERATIONS) {
    
    return intervalRootFinder(f, [](const Interval&) { return true; }, x0, tol, x, max_iterations);
}

} // namespace NexDynIPC::Physics::CCD
