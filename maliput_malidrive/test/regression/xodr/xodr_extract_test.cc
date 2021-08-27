// Copyright 2021 Toyota Research Institute
#include "maliput_malidrive/xodr/xodr_extract.h"

#include <string>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {
namespace test {
namespace {

// Template of a XODR description that contains only the xodr header.
constexpr const char* kXODRTemplate =
    R"R(<OpenDRIVE>
    <header revMajor="1" revMinor="4" name="" vendor="xodr_extract App"/>{}
</OpenDRIVE>
)R";

constexpr const char* kRoadNodeTemplate = R"R(
    <road name="Road {0}" length="2." id="{0}" junction="{1}">{2}
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="1.3" length="2.">
                <line/>
            </geometry>
        </planView>
        <lanes>
            <laneSection s="0.">
                <left>
                    <lane id="1" type="driving" level="false">
                        <width sOffset="0." a="1." b="2." c="3." d="4."/>
                    </lane>
                </left>
                <center>
                    <lane id="0" type="driving" level="false"/>
                </center>
            </laneSection>
        </lanes>
    </road>)R";

constexpr const char* kRoadLinkNode = R"R(
        <link>
            <predecessor elementType="road" elementId="5" contactPoint="end"/>
            <successor elementType="road" elementId="669" contactPoint="start"/>
        </link>)R";

class XodrExtractTestBase : public ::testing::Test {
 public:
  std::string GetRoadNode(const std::string& road_id, const std::string& junction_id,
                          const std::string& road_link_node) {
    return fmt::format(kRoadNodeTemplate, road_id, junction_id, road_link_node);
  }
  std::string GetXodrDescription(const std::vector<std::string>& road_nodes) {
    std::string all_roads{};
    for (const auto& road_node : road_nodes) {
      all_roads += road_node;
    }
    return fmt::format(kXODRTemplate, all_roads);
  }
};

TEST_F(XodrExtractTestBase, Throws) {
  tinyxml2::XMLDocument xodr_doc;
  xodr_doc.Parse(GetXodrDescription({GetRoadNode("1", "-1", "")}).c_str());

  // No Throws
  EXPECT_NO_THROW(XodrExtract(&xodr_doc, {"1"}, false));

  // Throws because `xodr_doc` argument is nullptr.
  EXPECT_THROW(XodrExtract(nullptr, {"1", "2"}, false), maliput::common::assertion_error);

  // Throws because `road_ids` is empty.
  EXPECT_THROW(XodrExtract(&xodr_doc, {}, false), maliput::common::assertion_error);
}

class XodrExtractTest : public XodrExtractTestBase {
 protected:
  void SetUp() override {
    xml_description_ =
        GetXodrDescription({GetRoadNode("1", "183", kRoadLinkNode), GetRoadNode("2", "-1", kRoadLinkNode),
                            GetRoadNode("3", "183", kRoadLinkNode)});
    xml_doc_.Parse(xml_description_.c_str());
  }
  std::string xml_description_{};

 protected:
  tinyxml2::XMLDocument xml_doc_;
};

TEST_F(XodrExtractTest, AllRoadsAreFound) {
  const std::string output_xodr = XodrExtract(&xml_doc_, {"1", "2", "3"}, false);
  EXPECT_EQ(xml_description_, output_xodr);
}

TEST_F(XodrExtractTest, ARoadIsMissing) {
  const std::string output_xodr = XodrExtract(&xml_doc_, {"1", "2", "3", "4"}, false);
  EXPECT_EQ(xml_description_, output_xodr);
}

// Linkage and junction information are updated.
TEST_F(XodrExtractTest, AllRoadsAreFoundUpdateLinkage) {
  const std::string output_xodr = XodrExtract(&xml_doc_, {"1", "2", "3"}, true);
  // Expected xml description: Junction id equal to -1 and road linkage is removed.
  const std::string expected_xml_description_ =
      GetXodrDescription({GetRoadNode("1", "-1", ""), GetRoadNode("2", "-1", ""), GetRoadNode("3", "-1", "")});
  EXPECT_EQ(expected_xml_description_, output_xodr);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
