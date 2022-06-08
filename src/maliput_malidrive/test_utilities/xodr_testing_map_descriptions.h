// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

namespace malidrive {
namespace test {

//@{ Testing maps
constexpr const char* kXodrSingleGeometry = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrSingleGeometry' version='1.0' date='Tue Sep 14 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="NoSimplification" length="10" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10">
              <line/>
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

constexpr const char* kXodrLineAndArcGeometry = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrLineAndArcGeometry' version='1.0' date='Tue Sep 14 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="NoSimplification" length="12" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10">
              <line/>
          </geometry>
          <geometry s="10.0" x="10.0" y="0.0" hdg="0.0" length="10">
              <arc curvature="0.1" />
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

constexpr const char* kXodrWithLinesToBeSimplified = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrWithLinesToBeSimplified' version='1.0' date='Tue Sep 14 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="SimplifiesLines" length="12" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="10.0" hdg="0.0" length="10">
              <line/>
          </geometry>
          <geometry s="10.0" x="10.0" y="10.0" hdg="0.0" length="1">
              <line/>
          </geometry>
          <geometry s="11.0" x="11.0" y="10.0" hdg="0.0" length="1">
              <line/>
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

constexpr const char* kXodrWithArcsToBeSimplified = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrWithArcsToBeSimplified' version='1.0' date='Tue Sep 14 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="SimplifiesArcs" length="32.41592653589793" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="15.707963267948966">
              <arc curvature="0.1" />
          </geometry>
          <geometry s="15.707963267948966" x="10.0" y="10.0" hdg="1.5707963267948966" length="15.707963267948966">
              <arc curvature="0.1" />
          </geometry>
          <geometry s="31.41592653589793" x="0.0" y="20.0" hdg="3.141592653589793" length="1">
              <arc curvature="0.1" />
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

constexpr const char* kXodrCombinedLinesWithArcs = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrCombinedLinesWithArcs' version='1.0' date='Tue Sep 14 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="CombinedLinesWithArcs" length="41.41592653589793" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="15.707963267948966">
              <arc curvature="0.1" />
          </geometry>
          <geometry s="15.707963267948966" x="10.0" y="10.0" hdg="1.5707963267948966" length="5">
              <line/>
          </geometry>
          <geometry s="20.707963267948966" x="10.0" y="15.0" hdg="1.5707963267948966" length="5">
              <line/>
          </geometry>
          <geometry s="25.707963267948966" x="0.0" y="20.0" hdg="1.5707963267948966" length="15.707963267948966">
              <arc curvature="0.1" />
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

constexpr const char* kXodrCombinedArcsWithLines = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrCombinedArcsWithLines' version='1.0' date='Tue Sep 14 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="CombinedArcsWithLines" length="51.41592653589793" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10">
              <line/>
          </geometry>
          <geometry s="10.0" x="10.0" y="0.0" hdg="0" length="15.707963267948966">
              <arc curvature="0.1" />
          </geometry>
          <geometry s="25.707963267948966" x="20.0" y="10.0" hdg="1.5707963267948966" length="15.707963267948966">
              <arc curvature="0.1" />
          </geometry>
          <geometry s="41.41592653589793" x="10.0" y="20.0" hdg="3.141592653589793" length="10">
              <line/>
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

constexpr const char* kTown04Road399 = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrCombinedArcsWithLines' version='1.0' date='Tue Sep 14 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="Road 399" length="1.9102111310660320e+1" id="399" junction="-1">
      <link/>
        <planView>
            <geometry s="0.0000000000000000e+0" x="3.1283780569538402e+2" y="1.8031339835661868e+2" hdg="-1.5797139021955537e+0" length="4.3068866785660020e-2">
                <line/>
            </geometry>
            <geometry s="4.3068866785660020e-2" x="3.1283742163060742e+2" y="1.8027033120230766e+2" hdg="-1.5797139021955537e+0" length="7.0006205424934933e+0">
                <line/>
            </geometry>
            <geometry s="7.0436894092791817e+0" x="3.1277499389648438e+2" y="1.7326998901367188e+2" hdg="-1.5797139021955546e+0" length="2.6432891455440028e+0">
                <line/>
            </geometry>
            <geometry s="9.6869785548231846e+0" x="3.1275142247863840e+2" y="1.7062680496877235e+2" hdg="-1.5797139021955546e+0" length="9.3738955918390445e+0">
                <line/>
            </geometry>
            <geometry s="1.9060874146662229e+1" x="3.1266783116581797e+2" y="1.6125328209532074e+2" hdg="-1.5797139021955546e+0" length="4.1237163998090409e-2">
                <line/>
            </geometry>
        </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";
//@}

}  // namespace test
}  // namespace malidrive
