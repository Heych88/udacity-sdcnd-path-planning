

#ifndef FILTER_H
#define FILTER_H

#include <vector>
#include <queue>

using namespace std;

// Moving average class for a Queue 
// Original Source code by mehuld https://discuss.leetcode.com/topic/45731/c-solution-using-a-queue-calculate-average-in-o-1
class MovingAverage {
private:
    queue<double> q;
    int queue_size;
    int sum;
public:
  /** Initialize your data structure here. */
  MovingAverage() 
  {
     sum = 0;
  }

  void setSize(const int size)
  {
    queue_size = size;
  }
  
  int getSize()
  {
    return q.size();
  }
  
  void emptyQueue()
  {
    while (!q.empty())
    {
      q.pop();
    }
    sum = 0;
  }

  double nextAverage(const int val) 
  {
    if(q.size() < queue_size)
    {
      q.push(val);
      sum = sum + val;
    }
    else if(q.size() == queue_size)
    {
      int top = q.front();
      sum = sum - top;
      q.pop();
      q.push(val);
      sum = sum + val;
    }

    if(q.size() > 0)
      return double(sum)/(q.size());
    else
      return 0;
  }
};

#endif /* FILTER_H */

