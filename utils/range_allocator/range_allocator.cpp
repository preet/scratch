#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <string>
#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <cassert>

using uint = unsigned int;

// TODO use forward_list instead?

template<typename T> // T should be copyable/movable(?), makes sense for it to be a reference or uid
class RangeAllocator
{
public:
    // 16 bytes
    struct Block;

    struct Range
    {
        uint start;
        uint size;
        typename std::list<Block>::iterator block;
    };

    struct Block
    {
        T data;
        std::vector<Range> list_avail; // sorted, make sure to reserve
        std::list<Range> list_used;
    };

    using BlockListIterator         = typename std::list<Block>::iterator;
    using BlockListConstIterator    = typename std::list<Block>::iterator;
    using RangeListIterator         = typename std::list<Range>::iterator;
    using RangeListConstIterator    = typename std::list<Range>::iterator;
    using RangeVectorIterator       = typename std::vector<Range>::iterator;

    //

    RangeAllocator(uint block_size) :
        m_block_size(block_size)
    {

    }

    ~RangeAllocator()
    {

    }

    BlockListConstIterator
    CreateBlock(T block_data)
    {
        m_list_blocks.push_back(
                    Block{
                        block_data,
                        {}, // list_avail
                        {}  // list_used
                    });

        // add the initial range
        m_list_blocks.back().list_avail.push_back(
                    Range{0,
                          m_block_size,
                          std::prev(m_list_blocks.end())});

        return std::prev(m_list_blocks.end());
    }

    T RemoveBlock(BlockListConstIterator it)
    {
        auto block_data = it->data;
        m_list_blocks.erase(it);

        return block_data;
    }

    RangeListConstIterator
    AcquireRange(uint size, bool &ok)
    {
        ok = false;

        if(size > m_block_size) {
            // print some error here
            RangeListConstIterator x;
            return x;
        }

        for(auto block_it = m_list_blocks.begin();
            block_it != m_list_blocks.end(); ++block_it)
        {
            Block &block = *block_it;

            for(auto range_it = block.list_avail.begin();
                range_it != block.list_avail.end(); ++range_it)
            {
                if(range_it->size > size)
                {
                    // split the range
                    Range range_used{
                        range_it->start,
                        size,
                        block_it
                    };

                    Range range_keep{
                        range_it->start+size,
                        range_it->size-size,
                        block_it
                    };

                    block.list_used.push_back(range_used);
                    block.list_avail.erase(range_it);

                    // ordered insert of split range
                    listAvailOrderedInsert(block.list_avail,range_keep);

                    ok = true;
                    return std::prev(block.list_used.end());
                }
                else if(range_it->size == size)
                {
                    // no need to split the range,
                    // just move it to the used list
                    block.list_used.push_back(*range_it);
                    block.list_avail.erase(range_it);

                    ok = true;
                    return std::prev(block.list_used.end());
                }
            }
        }

        // If we get here it means all blocks are full
        RangeListConstIterator x;
        return x;
    }

    BlockListConstIterator
    ReleaseRange(RangeListConstIterator range, bool &empty)
    {
        auto block = range->block;

        // ordered insert back into list_avail
        listAvailOrderedInsert(
                    block->list_avail,
                    *range);

        // remove from list_used
        block->list_used.erase(range);

        if(block->list_used.empty()) {
            empty = true;
            return block;
        }
        else {
            empty = false;
            BlockListConstIterator null_it;
            return null_it;
        }
    }

private:
    void listAvailOrderedInsert(std::vector<Range> &list_avail,
                                Range range)
    {
        auto it = std::upper_bound(
                    list_avail.begin(),
                    list_avail.end(),
                    range,
                    [](Range const &a, Range const &b){
                        return (a.start < b.start);
                    });

        if(it != list_avail.end())
        {
            // it points to a range with a start value
            // greater than range

            // see if we can merge with the next range
            if(it->start == (range.start+range.size)) {
                // we can merge with the next range (it)
                it->start = range.start;
                it->size += range.size;

                // see if we can merge with the preceding range
                if(it != list_avail.begin()) {
                    auto it_prev = std::prev(it);
                    if(it->start == (it_prev->start+it_prev->size)) {
                        // we can merge with the prev range (it_prev)
                        it->start = it_prev->start;
                        it->size += it_prev->size;

                        list_avail.erase(it_prev);
                    }
                }
            }
            // see if we can merge with the preceding range
            else if(it != list_avail.begin()) {
                auto it_prev = std::prev(it);
                if(range.start == (it_prev->start+it_prev->size)) {
                    // we can merge with the prev range (it_prev)
                    it_prev->size += range.size;
                }
            }
            else {
                // can't merge
                list_avail.insert(it,range);
            }
        }
        else {
            list_avail.insert(it,range);
        }
    }

    //
    uint const m_block_size;
    std::list<Block> m_list_blocks;
};

TEST_CASE("RangeAllocator","[rangeallocator]")
{
    SECTION("Construction")
    {
        RangeAllocator<uint> rac(100);

        SECTION("Acquire Range / No Blocks")
        {
            bool ok;
            rac.AcquireRange(10,ok);
            REQUIRE(ok==false);
        }

        SECTION("Create Block")
        {
            auto it_b0 = rac.CreateBlock(0);
            REQUIRE(it_b0->data == 0);

            SECTION("Acquire Range > Block Size")
            {
                bool ok;
                rac.AcquireRange(1000,ok);
                REQUIRE(ok==false);
            }

            SECTION("Acquire Range == Block Size")
            {
                bool ok;
                auto it0 = rac.AcquireRange(100,ok);
                REQUIRE(ok);
                REQUIRE(it0->start == 0);
                REQUIRE(it0->size == 100);
                REQUIRE(it0->block->data == 0);

                // should be filled up
                rac.AcquireRange(100,ok);
                REQUIRE(ok==false);
            }

            SECTION("Acquire Range < Block Size")
            {
                bool ok;
                auto it0 = rac.AcquireRange(25,ok);
                REQUIRE(ok);
                REQUIRE(it0->start == 0);
                REQUIRE(it0->size == 25);
                REQUIRE(it0->block->data == 0);

                auto it1 = rac.AcquireRange(25,ok);
                REQUIRE(ok);
                REQUIRE(it1->start == 25);
                REQUIRE(it1->size == 25);
                REQUIRE(it1->block->data == 0);

                // shouldn't have enough space
                rac.AcquireRange(75,ok);
                REQUIRE(ok==false);

                auto it2 = rac.AcquireRange(50,ok);
                REQUIRE(ok);
                REQUIRE(it2->start == 50);
                REQUIRE(it2->size == 50);
                REQUIRE(it2->block->data == 0);

                // should be filled up
                rac.AcquireRange(1,ok);
                REQUIRE(ok==false);

                SECTION("Release Range")
                {
                    REQUIRE(it_b0->list_avail.size()==0);

                    bool empty;
                    rac.ReleaseRange(it2,empty);
                    REQUIRE(empty==false);
                    REQUIRE(it_b0->list_avail.size()==1);
                    REQUIRE(it_b0->list_avail.begin()->start == 50);
                    REQUIRE(it_b0->list_avail.begin()->size == 50);

                    // disjoint ranges shouldn't merge
                    rac.ReleaseRange(it0,empty);
                    REQUIRE(empty==false);
                    REQUIRE(it_b0->list_avail.size()==2);
                    REQUIRE(it_b0->list_avail.begin()->start == 0);
                    REQUIRE(it_b0->list_avail.begin()->size == 25);

                    // adjacent ranges should be merged
                    it0 = rac.AcquireRange(25,ok);
                    REQUIRE(it_b0->list_avail.size()==1);
                    REQUIRE(it_b0->list_avail.begin()->start == 50);
                    REQUIRE(it_b0->list_avail.begin()->size == 50);

                    rac.ReleaseRange(it1,empty);
                    REQUIRE(it_b0->list_avail.size()==1);
                    REQUIRE(it_b0->list_avail.begin()->start == 25);
                    REQUIRE(it_b0->list_avail.begin()->size == 75);

                    auto itf = rac.AcquireRange(75,ok);
                    REQUIRE(it_b0->list_avail.size()==0);

                    // check that empty flag is set when
                    // the block is completely emptied
                    rac.ReleaseRange(it0,empty);
                    REQUIRE(empty==false);

                    rac.ReleaseRange(itf,empty);
                    REQUIRE(empty);
                }
            }
        }
    }
}
