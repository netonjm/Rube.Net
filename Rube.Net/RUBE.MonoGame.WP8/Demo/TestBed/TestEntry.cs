using System;

namespace FarseerPhysics.TestBed.Framework
{
    public struct TestEntry
    {
        public Func<Test> CreateTest;
        public Func<Test> CreateTestRube;
        public string Name;
    }
}