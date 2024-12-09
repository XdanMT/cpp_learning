// 本demo实现了基类的Start方法和Stop方法的多态，
// 编译方法: g++ multiprocess_polymophism_demo.cpp -o test -lpthread

#include <iostream>
#include <thread>
#include <memory>
#include <atomic>
#include <chrono>  
using namespace std;


class BaseModule {
public:

    virtual void Run() = 0;
    virtual string GetSonName() = 0;

    int32_t Start(void (BaseModule::*Run)(void)) {
    stop_ = false;
    produce_thread_ = std::make_shared<std::thread>(&BaseModule::Run, this);
    if (!produce_thread_) {
        cout << "Start thread failed." << endl;
        return -1;
    }
    cout << "thread " << GetSonName() << " start." << endl;
    return 0;
  }

    int32_t Stop(std::shared_ptr<std::thread> produce_thread) {
    stop_ = true;
    if (produce_thread) {
      if (produce_thread->joinable()) {
        produce_thread->join();     // Stop()线程会在此行阻塞,等待produce_thread子线程函数执行结束返回后，再执行后面的语句！！
        cout << "Join the " << GetSonName() << "thread." << endl;
      }
    }
    cout << "thread "<< GetSonName() << " stop." << endl << endl;
    return 0;
  }


public:
    std::shared_ptr<std::thread> produce_thread_;
    std::atomic<bool> stop_;

};


class PreprocessModule : public BaseModule {
public:

    PreprocessModule() : _count(0), _name("preprocess_block"){}


    void Run() override{
        while (!stop_)
        {
            cout << "Preprocess frame: " << _count << endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            this->_count += 1;
        }
        if (stop_)
        {
            cout << "Thread " << _name << " is going to be over!" << endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    };

    virtual string GetSonName() override {
        return _name;
    }

private:
    int32_t _count;
    string _name;
};


class InferModule : public BaseModule {
public:

    InferModule() : _count(10), _name("infer_block"){}


    void Run() override{
        while (!stop_)
        {
            cout << "Infering frame: " << _count << endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            this->_count += 1;
        }
        if (stop_)
        {
            cout << "Thread " << _name << " is going to be over!" << endl;
            std::this_thread::sleep_for(std::chrono::seconds(4));
        }
    };

    virtual string GetSonName() override {
        return _name;
    }

private:
    int32_t _count;
    string _name;
};

int main(){
    PreprocessModule preprocess_ob;
    InferModule infer_ob;
    auto preprocess_module = &preprocess_ob;
    auto infer_module = &infer_ob;
    preprocess_module->Start(&BaseModule::Run);
    infer_module->Start(&BaseModule::Run);
    std::this_thread::sleep_for(std::chrono::seconds(5));   

    preprocess_module->Stop(preprocess_module->produce_thread_);   
    infer_module->Stop(infer_module->produce_thread_);   



    return 0;
}