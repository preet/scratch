#include "helper_testkssigslotspeeds.h"

#include <cassert>

HelperTestKsSigSlotSpeed::HelperTestKsSigSlotSpeed(QQuickView *qqview, QObject *parent) :
    Helper(qqview,parent)
{

}

void HelperTestKsSigSlotSpeed::runTest()
{
    using namespace ks;

    // emitter
    shared_ptr<Thread> thread_e = make_shared<Thread>();
    shared_ptr<test::Emitter> emitter = make_shared<test::Emitter>(thread_e);

    // receiver
    shared_ptr<Thread> thread_r = make_shared<Thread>();
    shared_ptr<test::Receiver> r0 = make_shared<test::Receiver>(thread_r);

    // ============================================================= //

    size_t const one_one_count=1000;

    // test 1 signal -> 1 slot

    Thing thing;
    thread_r->Start();

    emitter->SignalThing.Connect(r0,&test::Receiver::SlotThing);
    emitter->SignalStopThread.Connect(r0,&test::Receiver::SlotStopThread);

    std::chrono::time_point<std::chrono::steady_clock> start,end;
    start = std::chrono::steady_clock::now();

    for(size_t i=0; i < one_one_count; i++) {
        emitter->SignalThing.Emit(thing);
    }
    emitter->SignalStopThread.Emit(thread_r.get());
    thread_r->Wait();

    end = std::chrono::steady_clock::now();
    std::chrono::microseconds elapsed_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(
                end-start);

    std::string message0 = "1:1 signal:slot mapping:\n"
                           "1000 signals took: " +
            to_string(elapsed_ms.count()) + " us";

    assert(r0->invoke_count == one_one_count);

    // ============================================================= //

    // reset count
    r0->invoke_count = 0;

    // test 1 signal -> 4 slots
    shared_ptr<test::Receiver> r1 = make_shared<test::Receiver>(thread_r);
    shared_ptr<test::Receiver> r2 = make_shared<test::Receiver>(thread_r);
    shared_ptr<test::Receiver> r3 = make_shared<test::Receiver>(thread_r);

    size_t const one_many_count=1000;

    thread_r->Start();

    emitter->SignalThing.Connect(r1,&test::Receiver::SlotThing);
    emitter->SignalThing.Connect(r2,&test::Receiver::SlotThing);
    emitter->SignalThing.Connect(r3,&test::Receiver::SlotThing);

    start = std::chrono::steady_clock::now();

    for(size_t i=0; i < one_many_count; i++) {
        emitter->SignalThing.Emit(thing);
    }
    emitter->SignalStopThread.Emit(thread_r.get());
    thread_r->Wait();

    end = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration_cast<
            std::chrono::microseconds>(end-start);

    std::string message1 = "\n\n1:4 signal:slot mapping:\n"
                           "1000 signals took: " +
            to_string(elapsed_ms.count()) + " us";

    assert((r0->invoke_count +
            r1->invoke_count +
            r2->invoke_count +
            r3->invoke_count) == (one_many_count*4));

    // ============================================================= //

    emit testComplete(QString::fromStdString(message0+message1));
}
